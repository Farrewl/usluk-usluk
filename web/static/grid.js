// web/static/grid.js — peta lokal grid A1..E5 (25x25 m) + trajectory realtime.
window.GridMap = (function () {
  const R = 111320; // meter per derajat lintang

  function create(canvas, compass) {
    const ctx = canvas.getContext('2d');
    const cctx = compass ? compass.getContext('2d') : null;
    let cfg = {
      anchor: null, cell: 5, cols: 5, rows: 5,
      waypoints: [], visionLegs: [],
    };
    let trail = [];
    let pose = { lat: null, lon: null, yaw: 0 };
    const MAX_TRAIL = 1200;

    function setConfig(c) {
      cfg = Object.assign(cfg, c || {});
      if (cfg.waypoints && cfg.waypoints.length && cfg.anchor == null) {
        cfg.anchor = cfg.waypoints[0];
      }
      trail = [];
      draw();
    }

    function setAnchor(a) { if (a) { cfg.anchor = a; trail = []; draw(); } }

    function toLocal(lat, lon) {
      const a = cfg.anchor;
      if (!a || lat == null || lon == null) return null;
      const mLon = R * Math.cos(a.lat * Math.PI / 180);
      return { x: (lon - a.lon) * mLon, y: (lat - a.lat) * R };
    }

    // meter -> piksel (grid 0..cols*cell)
    function mapper() {
      const spanX = cfg.cols * cfg.cell, spanY = cfg.rows * cfg.cell;
      const pad = 34;
      const sx = (canvas.width - 2 * pad) / spanX;
      const sy = (canvas.height - 2 * pad) / spanY;
      return {
        pad, spanX, spanY,
        px: (mx) => pad + mx * sx,
        py: (my) => canvas.height - pad - my * sy,
      };
    }

    function draw() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      const m = mapper();
      const letters = 'ABCDEFGH';

      // sel grid + label
      for (let c = 0; c < cfg.cols; c++) {
        for (let r = 0; r < cfg.rows; r++) {
          const x0 = m.px(c * cfg.cell), x1 = m.px((c + 1) * cfg.cell);
          const y0 = m.py((r + 1) * cfg.cell), y1 = m.py(r * cfg.cell);
          ctx.fillStyle = (c + r) % 2 ? '#111820' : '#0f151d';
          ctx.fillRect(x0, y0, x1 - x0, y1 - y0);
          ctx.strokeStyle = '#24303d';
          ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
          ctx.fillStyle = '#43566b';
          ctx.font = 'bold 13px monospace';
          ctx.fillText(letters[c] + (r + 1), x0 + 6, y1 - 6);
        }
      }

      // waypoint + leg vision
      cfg.waypoints.forEach((wp, i) => {
        const p = toLocal(wp.lat, wp.lon);
        if (!p) return;
        const x = m.px(p.x), y = m.py(p.y);
        const vision = (cfg.visionLegs || []).includes(i + 1);
        ctx.fillStyle = vision ? '#3fb950' : '#58a6ff';
        ctx.beginPath();
        ctx.arc(x, y, vision ? 6 : 4, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = '#8b949e';
        ctx.font = '10px monospace';
        ctx.fillText('WP' + (i + 1), x + 8, y - 6);
      });

      // trajectory
      if (trail.length > 1) {
        ctx.strokeStyle = '#3fb950';
        ctx.lineWidth = 2;
        ctx.beginPath();
        trail.forEach((pt, i) => {
          const p = toLocal(pt.lat, pt.lon);
          if (!p) return;
          const x = m.px(p.x), y = m.py(p.y);
          if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        });
        ctx.stroke();
        ctx.lineWidth = 1;
      }

      // kapal
      const bp = toLocal(pose.lat, pose.lon);
      if (bp) {
        const x = m.px(bp.x), y = m.py(bp.y);
        ctx.save();
        ctx.translate(x, y);
        ctx.rotate((pose.yaw || 0) * Math.PI / 180);
        ctx.fillStyle = '#58a6ff';
        ctx.beginPath();
        ctx.moveTo(0, -11); ctx.lineTo(7, 9); ctx.lineTo(0, 4); ctx.lineTo(-7, 9);
        ctx.closePath(); ctx.fill();
        ctx.restore();
      }

      drawCompass();
    }

    function drawCompass() {
      if (!cctx) return;
      const s = compass.width, c = s / 2, r = c - 8;
      cctx.clearRect(0, 0, s, s);
      cctx.strokeStyle = '#24303d'; cctx.lineWidth = 2;
      cctx.beginPath(); cctx.arc(c, c, r, 0, Math.PI * 2); cctx.stroke();
      cctx.save();
      cctx.translate(c, c);
      cctx.rotate(-(pose.yaw || 0) * Math.PI / 180);
      cctx.fillStyle = '#f85149';
      cctx.beginPath(); cctx.moveTo(0, -r + 6); cctx.lineTo(5, 6); cctx.lineTo(-5, 6);
      cctx.closePath(); cctx.fill();
      cctx.fillStyle = '#e6edf3';
      cctx.beginPath(); cctx.moveTo(0, r - 6); cctx.lineTo(5, -4); cctx.lineTo(-5, -4);
      cctx.closePath(); cctx.fill();
      cctx.restore();
      cctx.fillStyle = '#8b949e'; cctx.font = 'bold 12px monospace';
      cctx.fillText('N', c - 4, 14);
      cctx.fillText('UTARA', c - 20, s - 4);
    }

    function push(lat, lon, yaw) {
      if (lat == null) return;
      pose = { lat: lat, lon: lon, yaw: yaw || 0 };
      trail.push({ lat, lon });
      if (trail.length > MAX_TRAIL) trail.shift();
      draw();
    }

    window.addEventListener('resize', draw);
    return { setConfig, setAnchor, push, draw };
  }

  return { create };
})();
