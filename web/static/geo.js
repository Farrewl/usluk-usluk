// web/static/geo.js — format geo-tag & satuan (dipakai dashboard.js & popup foto).
window.Geo = (function () {
  const DAYS = ['Sun', 'Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat'];

  function pad(n) { return String(n).padStart(2, '0'); }

  function stamp(ts) {
    const d = new Date((ts || Date.now() / 1000) * 1000);
    return {
      day: DAYS[d.getDay()],
      date: pad(d.getDate()) + '/' + pad(d.getMonth() + 1) + '/' + d.getFullYear(),
      time: pad(d.getHours()) + ':' + pad(d.getMinutes()) + ':' + pad(d.getSeconds()),
    };
  }

  function hemi(v, isLat) {
    if (isLat) return v >= 0 ? 'N' : 'S';
    return v >= 0 ? 'E' : 'W';
  }

  function dd(lat, lon) {
    if (lat == null || lon == null) return '--';
    return hemi(lat, true) + ' ' + Math.abs(lat).toFixed(5) +
      ' ' + hemi(lon, false) + ' ' + Math.abs(lon).toFixed(5);
  }

  function dmStr(v, isLat) {
    const a = Math.abs(v);
    const d = Math.floor(a);
    const m = ((a - d) * 60).toFixed(4).replace('.', ',');
    return hemi(v, isLat) + ' ' + d + '\u00b0 ' + m + "'";
  }

  function dm(lat, lon) {
    if (lat == null || lon == null) return '--';
    return dmStr(lat, true) + ' ' + dmStr(lon, false);
  }

  const MPS_TO_KN = 1.943844;
  const MPS_TO_KMH = 3.6;

  return { stamp, dd, dm, MPS_TO_KN, MPS_TO_KMH };
})();
