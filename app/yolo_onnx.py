"""app/yolo_onnx.py — inferensi YOLOv8 via `cv2.dnn` (ONNX), TANPA torch.

Kenapa: `ultralytics`+`torch` memakan ~250 MB RAM dan ~1–3 s waktu import,
padahal `opencv-python-headless` (sudah dipakai untuk kamera & validasi)
mampu menjalankan model ONNX export yang sama. Hasilnya: RAM & startup jauh
lebih ringan, dependensi lebih sedikit — tanpa mengubah pipeline visi.

Kompatibel dengan jalur lama: model dipanggil sama seperti ultralytics,

    res = model(frame, conf=0.25, imgsz=256, verbose=False)
    res[0].boxes            # iterable: tiap item punya .cls/.xyxy/.conf (list)
    res[0].names            # {int: nama_kelas}

Preprocessing & postprocessing ditulis persis seperti ultralytics
(letterbox 114, BGR->RGB, /255, xywh->xyxy, NMS) agar deteksi identik dengan
jalur `.pt`/torch. Kelas & output = YOLOv8 `(1, 4+nc, N)`.
"""

import cv2
import numpy as np

_LETTERBOX_COLOR = (114, 114, 114)


class _BoxItem:
    """Satu deteksi; meniru Box ultralytics (atribut list agar `[0]` jalan)."""

    __slots__ = ("cls", "xyxy", "conf")

    def __init__(self, cls, xyxy, conf):
        self.cls = [int(cls)]
        self.xyxy = [[float(v) for v in xyxy]]
        self.conf = [float(conf)]


class _Boxes:
    def __init__(self, items):
        self._items = items

    def __len__(self):
        return len(self._items)

    def __iter__(self):
        return iter(self._items)


class _Result:
    def __init__(self, items, names):
        self.boxes = _Boxes(items)
        self.names = names
        self.orig_shape = None

    def plot(self):
        return None


def _letterbox(img, new_shape, stride=32):
    """Letterbox ala ultralytics predict (`auto=True`, stride 32).

    Bedanya dengan letterbox persegi biasa: sisi pendek dipad hanya seperlunya
    agar kelipatan `stride` (minimum-rectangle), jadi input bisa NON-persegi
    (mis. 640x480 + imgsz=256 -> 192x256). Ini penting agar deteksi & skor
    identik dengan jalur ultralytics yang lama.
    """
    h, w = img.shape[:2]
    r = min(new_shape[0] / h, new_shape[1] / w)
    new_unpad = (int(round(w * r)), int(round(h * r)))
    dw = np.mod(new_shape[1] - new_unpad[0], stride)
    dh = np.mod(new_shape[0] - new_unpad[1], stride)
    dw /= 2
    dh /= 2
    if (w, h) != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right,
                             cv2.BORDER_CONSTANT, value=_LETTERBOX_COLOR)
    return img, r, (left, top)


def _nms(boxes_xyxy, scores, conf_thres, iou_thres):
    if len(boxes_xyxy) == 0:
        return []
    xywh = []
    for x1, y1, x2, y2 in boxes_xyxy:
        xywh.append([int(x1), int(y1), int(x2 - x1), int(y2 - y1)])
    idx = cv2.dnn.NMSBoxes(xywh, [float(s) for s in scores],
                           float(conf_thres), float(iou_thres))
    if idx is None or len(idx) == 0:
        return []
    return [int(i) for i in np.array(idx).reshape(-1)]


class OnnxDetector:
    """Wrapper YOLOv8-ONNX via cv2.dnn, API mirip ultralytics YOLO."""

    def __init__(self, path, names, iou_thres=0.7):
        self.path = path
        self.names = {int(k): str(v) for k, v in (names or {}).items()}
        self.iou_thres = float(iou_thres)
        self.net = cv2.dnn.readNetFromONNX(path)
        try:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        except Exception:
            pass

    def __call__(self, frame, conf=0.25, imgsz=256, verbose=False, **kw):
        fh, fw = frame.shape[:2]
        size = int(imgsz or 256)
        lb, ratio, (pad_x, pad_y) = _letterbox(frame, (size, size))
        lb_h, lb_w = lb.shape[:2]
        blob = cv2.dnn.blobFromImage(lb, 1.0 / 255.0, (lb_w, lb_h),
                                     swapRB=True, crop=False)
        self.net.setInput(blob)
        out = self.net.forward()
        arr = np.squeeze(out, axis=0).T          # (N, 4+nc)
        if arr.shape[1] < 5:
            arr = np.squeeze(out, axis=0)         # (4+nc, N) fallback
            if arr.shape[0] < arr.shape[1]:
                arr = arr.T

        boxes = arr[:, :4]
        scores_all = arr[:, 4:]
        if scores_all.shape[1] == 1:
            cls_ids = np.zeros(len(arr), dtype=int)
            confs = scores_all[:, 0]
        else:
            cls_ids = np.argmax(scores_all, axis=1)
            confs = np.max(scores_all, axis=1)

        keep_conf = np.where(confs >= float(conf))[0]
        xyxy = []
        sc = []
        cl = []
        for i in keep_conf:
            cx, cy, w, h = boxes[i]
            # skala balik dari ruang letterbox ke frame asli
            x1 = (cx - w / 2 - pad_x) / ratio
            y1 = (cy - h / 2 - pad_y) / ratio
            x2 = (cx + w / 2 - pad_x) / ratio
            y2 = (cy + h / 2 - pad_y) / ratio
            x1 = max(0, min(fw - 1, x1))
            y1 = max(0, min(fh - 1, y1))
            x2 = max(0, min(fw - 1, x2))
            y2 = max(0, min(fh - 1, y2))
            xyxy.append((x1, y1, x2, y2))
            sc.append(float(confs[i]))
            cl.append(int(cls_ids[i]))

        kept = _nms(xyxy, sc, conf, self.iou_thres)
        items = [_BoxItem(cl[i], xyxy[i], sc[i]) for i in kept]
        return [_Result(items, self.names)]
