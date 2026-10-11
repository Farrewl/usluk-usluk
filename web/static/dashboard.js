// web/static/dashboard.js — SSE client dashboard monitoring KIT KKI.
(function () {
  const $ = (id) => document.getElementById(id);
  let grid = null;
  let cfg = null;
  let lastPhotoTs = { surface: 0, under: 0 };

  function initGrid() {
    grid = GridMap.create($('grid'), $('compass'));
    grid.setConfig({
      anchor: cfg.track_start || (cfg.waypoints && cfg.waypoints[0]) || null,
      cell: cfg.grid.cell_m, cols: cfg.grid.cols, rows: cfg.grid.rows,
      waypoints: cfg.waypoints || [], visionLegs: cfg.vision_legs || [],
    });
  }

  function setTrack(name) {
    fetch('/config?track=' + name).then(r => r.json()).then(c => {
      cfg = c;
      $('trackBadge').textContent = 'START (' +
        (c.track_start ? c.track_start.color.toUpperCase() : '?') + ') / FINISH';
      if (grid) {
        grid.setAnchor(c.track_start || (c.waypoints && c.waypoints[0]) || null);
        grid.setConfig({
          cell: c.grid.cell_m, cols: c.grid.cols, rows: c.grid.rows,
          waypoints: c.waypoints || [], visionLegs: c.vision_legs || [],
        });
      }
    }).catch(() => {});
  }

  function renderPhoto(kind, info) {
    const box = kind === 'surface' ? $('photoSurface') : $('photoUnder');
    if (!box || !info) return;
    const body = box.querySelector('.ph-body');
    const geo = box.querySelector('.ph-geo');
    body.classList.remove('empty');
    body.innerHTML = '<img src="' + info.url + '?t=' + Date.now() + '" alt="' + kind + '">';
    const s = Geo.stamp(info.ts);
    geo.innerHTML =
      'Day: ' + s.day + ' &nbsp; Date: ' + s.date + ' &nbsp; Time: ' + s.time + '<br>' +
      'Coordinate DD: ' + (info.coord_dd || Geo.dd(info.lat, info.lon)) + '<br>' +
      "Coordinate DM: " + (info.coord_dm || Geo.dm(info.lat, info.lon)) + '<br>' +
      'SOG: ' + Number(info.sog_kn).toFixed(2) + ' kn / ' +
      Number(info.sog_kmh).toFixed(2) + ' km/h &nbsp; COG: ' +
      Number(info.cog_deg).toFixed(1) + '\u00b0';
  }

  function updateSteps(m) {
    if (!m) return;
    const set = (step, done) => {
      const li = document.querySelector('.steps li[data-step="' + step + '"]');
      if (li) li.classList.toggle('done', !!done);
    };
    set('prep', m.prep);
    set('start', m.start);
    set('balls', m.balls >= 10);
    set('surface', m.surface);
    set('under', m.under);
    set('dock', m.dock);
    $('ballsTick').textContent = m.balls + ' / 10';
    $('surfTick').textContent = m.surface ? 'DONE' : '-';
    $('underTick').textContent = m.under ? 'DONE' : '-';
  }

  function onData(d) {
    if (d.team) $('teamName').textContent = d.team;
    if (d.pt) $('teamPt').textContent = d.pt;

    const batt = d.battery_pct;
    $('batt').textContent = (batt == null) ? '--' : Number(batt).toFixed(0) + '%';
    $('batt').style.color = batt == null ? '' :
      batt >= 50 ? 'var(--ok)' : batt >= 20 ? 'var(--warn)' : 'var(--bad)';
    $('state').textContent = d.state || '--';
    $('link').textContent = d.mavlink_ok ? 'LINK' : 'NO-LINK';
    $('link').style.color = d.mavlink_ok ? 'var(--ok)' : 'var(--bad)';

    $('sogKn').textContent = Number(d.sog_kn || 0).toFixed(2) + ' kn';
    $('sogKmh').textContent = Number(d.sog_kmh || 0).toFixed(2) + ' km/h';
    $('cog').textContent = Number(d.cog_deg || 0).toFixed(1) + '\u00b0';
    $('yaw').textContent = 'H ' + Number(d.yaw_deg || 0).toFixed(1) + '\u00b0';
    $('att').textContent = 'P' + Number(d.pitch_deg || 0).toFixed(1) +
      ' R' + Number(d.roll_deg || 0).toFixed(1);
    $('wp').textContent = 'WP ' + ((d.wp_idx || 0) + 1) + '/' + (d.wp_total || '-');
    $('pos').textContent = (d.lat == null) ? '--' :
      Geo.dd(d.lat, d.lon);

    updateSteps(d.mission);
    if (grid) grid.push(d.lat, d.lon, d.cog_deg);

    if (d.last_photo) {
      const k = d.last_photo.kind;
      const key = k === 'under' ? 'under' : 'surface';
      if (d.last_photo.ts > lastPhotoTs[key]) {
        lastPhotoTs[key] = d.last_photo.ts;
        renderPhoto(key, d.last_photo);
      }
    }
  }

  function connect() {
    const es = new EventSource('/events');
    const badge = $('liveBadge');
    es.onopen = () => { badge.classList.remove('off'); badge.textContent = '\u25cf LIVE'; };
    es.onerror = () => { badge.classList.add('off'); badge.textContent = '\u25cf OFFLINE'; };
    es.onmessage = (ev) => { try { onData(JSON.parse(ev.data)); } catch (e) {} };
  }

  fetch('/config').then(r => r.json()).then(c => {
    cfg = c;
    initGrid();
    $('teamName').textContent = c.team;
    $('teamPt').textContent = c.pt;
    const sel = $('trackSelect');
    sel.value = c.track || 'A';
    $('trackBadge').textContent = 'START (' +
      (c.track_start ? c.track_start.color.toUpperCase() : '?') + ') / FINISH';
    sel.addEventListener('change', () => setTrack(sel.value));
    connect();
  }).catch(connect);

  // daftar capture awal (histori) supaya foto lama tetap tampil setelah reload
  fetch('/captures').then(r => r.json()).then(x => {
    (x.captures || []).slice().reverse().forEach(info => {
      const key = info.kind === 'under' ? 'under' : 'surface';
      if ((info.ts || 0) > lastPhotoTs[key]) {
        lastPhotoTs[key] = info.ts || 0;
        renderPhoto(key, info);
      }
    });
  }).catch(() => {});
})();
