#include "panel_page.h"

#include <string>

#include "glonass/types.h"
#include "request_params_l1oc.h"

namespace glonass_service {
namespace {
// Часть страницы до блока числовых констант сценария
constexpr const char* pageHead = R"PANEL(<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Панель модели сигнала L1OC</title>
<style>
:root{
  --bg:#eef1f4; --card:#ffffff; --fg:#10151c; --muted:#55606e;
  --line:#dfe4ea; --soft:#f0f3f6; --accent:#123f8f; --warn:#c2410c; --mark:#6b21a8;
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);
     font:14px/1.5 "Helvetica Neue",Helvetica,Arial,sans-serif}
header{background:var(--card);border-bottom:1px solid var(--line);padding:14px 20px}
h1{font-size:19px;font-weight:600;margin:0 0 3px}
.origin{margin:0 0 10px;color:var(--muted);font-size:12px}
h2{font-size:12px;font-weight:600;letter-spacing:.04em;text-transform:uppercase;
   color:var(--muted);margin:0 0 10px}
.segmented{display:flex;flex-wrap:wrap;gap:6px}
.segmented button{font:inherit;font-size:13px;padding:6px 12px;border:1px solid var(--line);
  background:var(--card);color:var(--fg);border-radius:6px;cursor:pointer}
.segmented button:hover{border-color:var(--accent)}
.segmented button.on{background:var(--accent);border-color:var(--accent);color:#fff}

main{display:grid;gap:14px;padding:14px 20px;align-content:start}
body.layout-overview main{grid-template-columns:300px minmax(0,1fr);
  grid-template-areas:"controls metrics" "controls grid"}
body.layout-detail main{grid-template-columns:minmax(0,1fr);
  grid-template-areas:"metrics" "tabs" "single" "controls"}
body.layout-compare main{grid-template-columns:minmax(0,1fr);
  grid-template-areas:"controls" "tabs" "compare"}
#controls{grid-area:controls}
#metrics{grid-area:metrics}
#tabs{grid-area:tabs}
#grid{grid-area:grid}
#single{grid-area:single}
#compare{grid-area:compare}
body.layout-overview #tabs,body.layout-overview #single,body.layout-overview #compare{display:none}
body.layout-detail #grid,body.layout-detail #compare{display:none}
body.layout-compare #grid,body.layout-compare #single,body.layout-compare #metrics{display:none}

#controls{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:14px}
.fields{display:grid;gap:13px}
body.layout-detail .fields,body.layout-compare .fields{
  grid-template-columns:repeat(auto-fit,minmax(240px,1fr));align-items:start}
.field label{display:flex;justify-content:space-between;gap:8px;font-size:12px;
  color:var(--muted);margin-bottom:4px}
.field .val{color:var(--fg);font-weight:600;white-space:nowrap}
.field input[type=range]{width:100%;accent-color:var(--accent);margin:0}
.check{display:flex;align-items:center;gap:6px;font-size:12px;color:var(--muted);
  margin-top:6px;cursor:pointer}
.check input{accent-color:var(--accent);margin:0}
.act{font:inherit;font-size:13px;padding:6px 12px;border:1px solid var(--line);
  background:var(--soft);color:var(--fg);border-radius:6px;cursor:pointer}
.act:hover{border-color:var(--accent)}

.tiles{display:grid;gap:8px;grid-template-columns:repeat(auto-fit,minmax(155px,1fr));
  align-content:start}
.tile{background:var(--card);border:1px solid var(--line);border-radius:6px;padding:7px 10px}
.tile .k{font-size:11px;color:var(--muted)}
.tile .v{font-size:16px;font-weight:600;line-height:1.3}
.tile.warn .v{color:var(--warn)}
.tile.wide{grid-column:1/-1}
.tile.wide .v{font-size:13px;font-weight:400;color:var(--warn)}

#grid{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(450px,1fr))}
.slot{background:var(--card);border:1px solid var(--line);border-radius:6px;padding:6px;
  min-height:140px;display:flex;align-items:center;justify-content:center}
.slot img{width:100%;height:auto;display:block;border-radius:4px}
.slot.busy{opacity:.5}
#grid .slot{cursor:pointer}
#single{display:grid;justify-items:center}
#single .slot{width:100%;max-width:972px}
.reject{color:var(--warn);font-size:13px;text-align:center;padding:24px 14px}
.hint{color:var(--muted);font-size:13px;text-align:center;padding:24px 14px}

#compare{display:grid;gap:12px;grid-template-columns:minmax(0,1fr) 230px minmax(0,1fr)}
#compare .col{display:grid;gap:8px;align-content:start}
.colhead{display:flex;align-items:center;justify-content:space-between;gap:8px;
  font-size:12px;font-weight:600;letter-spacing:.04em;text-transform:uppercase;color:var(--muted)}
.params{font-size:12px;color:var(--muted);background:var(--card);border:1px solid var(--line);
  border-radius:6px;padding:7px 10px}
#compare .tiles{grid-template-columns:repeat(auto-fit,minmax(140px,1fr))}
.col.diffs .tiles{grid-template-columns:minmax(0,1fr)}
.col.diffs .tile{border-color:var(--mark)}

@media (max-width:1100px){
  body.layout-overview main{grid-template-columns:minmax(0,1fr);
    grid-template-areas:"controls" "metrics" "grid"}
  #compare{grid-template-columns:minmax(0,1fr)}
}
</style>
</head>
<body class="layout-overview">
<header>
<h1>Панель модели навигационного сигнала L1OC</h1>
<p class="origin" id="origin">сведения о сервисе запрашиваются…</p>
<nav class="segmented" id="layouts">
<button type="button" data-layout="overview" class="on">Обзор</button>
<button type="button" data-layout="detail">Разбор</button>
<button type="button" data-layout="compare">Сравнение</button>
</nav>
</header>

<main>
<aside id="controls">
<h2>Параметры прогона</h2>
<div class="fields">
<div class="field">
<label for="jcount">Состав активных НКА<span class="val" id="jcountOut"></span></label>
<input type="range" id="jcount" min="1" max="24" step="1" value="24">
<label class="check"><input type="checkbox" id="jsingle"> только выбранный НКА</label>
</div>
<div class="field">
<label for="jpick">Выбранный НКА<span class="val" id="jpickOut"></span></label>
<input type="range" id="jpick" min="1" max="24" step="1" value="5">
</div>
<div class="field">
<label for="amp">Относительная амплитуда выбранного НКА<span class="val" id="ampOut"></span></label>
<input type="range" id="amp" min="0" max="1" step="0.05" value="1">
</div>
<div class="field">
<label for="fs">Частота дискретизации<span class="val" id="fsOut"></span></label>
<input type="range" id="fs" min="0" max="4" step="1" value="3">
</div>
<div class="field">
<label for="df">Расстройка опорной частоты<span class="val" id="dfOut"></span></label>
<input type="range" id="df" min="-50" max="50" step="1" value="0">
</div>
<div class="field">
<label for="tm">Модельное время<span class="val" id="tmOut"></span></label>
<input type="range" id="tm" min="0" max="50" step="1" value="0">
</div>
<div class="field">
<label for="phi">Начальная фаза несущей<span class="val" id="phiOut"></span></label>
<input type="range" id="phi" min="0" max="8" step="1" value="0">
</div>
<button type="button" class="act" id="reset">Опорные значения</button>
</div>
</aside>

<div class="tiles" id="metrics"></div>
<div class="segmented" id="tabs"></div>
<div id="grid"></div>
<div id="single"><div class="slot" id="slotOne"></div></div>

<div id="compare">
<section class="col">
<div class="colhead"><span>Опорная конфигурация</span>
<button type="button" class="act" id="pin">Взять текущую</button></div>
<div class="slot" id="slotRef"><div class="hint">опорная конфигурация не задана</div></div>
<div class="params" id="refParams">—</div>
<div class="tiles" id="refMetrics"></div>
</section>
<section class="col diffs">
<div class="colhead"><span>Разность</span></div>
<div class="tiles" id="diffMetrics"></div>
</section>
<section class="col">
<div class="colhead"><span>Текущая конфигурация</span></div>
<div class="slot" id="slotCur"></div>
<div class="params" id="curParams">—</div>
<div class="tiles" id="curMetrics"></div>
</section>
</div>
</main>

<script>
'use strict';
)PANEL";

// Часть страницы после блока числовых констант сценария
constexpr const char* pageTail = R"PANEL(
const kinds = [
  { id: 'psd',      tab: 'СПМ',           alt: 'Спектральная плотность мощности' },
  { id: 'waveform', tab: 'Осциллограмма', alt: 'Осциллограмма квадратур' },
  { id: 'acf',      tab: 'АКФ',           alt: 'Периодическая автокорреляционная функция дальномерного кода' },
  { id: 'ccf',      tab: 'ВКФ',           alt: 'Огибающая периодической взаимнокорреляционной функции ансамбля' },
  { id: 'navline',  tab: 'Строка НС',     alt: 'Кадр строки навигационного сообщения' },
  { id: 'level',    tab: 'Гистограмма',   alt: 'Гистограмма мгновенных значений' }
];
const lineTypeNames = { normal: 'нормальная', anomalous1: 'аномальная 1', anomalous2: 'аномальная 2' };
const subDigits = '₀₁₂₃₄₅₆₇₈₉';

let layout = 'overview';
let selectedKind = 'psd';
let referenceQuery = null;
let referenceState = null;
let referenceParams = null;
let currentState = null;
let frameToken = 0;
let stateTimer = 0;

function el(id) { return document.getElementById(id); }

// Русский числовой формат кадров: запятая, узкий неразрывный пробел в разрядах, знак U+2212
function numberRu(value, digits) {
  const sign = (value < 0) ? '−' : '';
  const text = Math.abs(value).toFixed((digits === undefined) ? 0 : digits);
  const parts = text.split('.');
  const whole = parts[0].replace(/\B(?=(\d{3})+(?!\d))/g, ' ');
  return sign + whole + ((parts.length > 1) ? (',' + parts[1]) : '');
}

function subscript(value) {
  return String(value).replace(/[0-9]/g, function (digit) { return subDigits[+digit]; });
}

function phiLabel(index) {
  if (index === 0) { return '0'; }
  let numerator = index;
  let denominator = 16;
  while (((numerator % 2) === 0) && ((denominator % 2) === 0)) { numerator /= 2; denominator /= 2; }
  return ((numerator === 1) ? 'π' : (numerator + 'π')) + ((denominator === 1) ? '' : ('/' + denominator));
}

function amplitudeText(value) { return String(+value.toFixed(2)); }

function kindOf(id) {
  for (let i = 0; i < kinds.length; ++i) { if (kinds[i].id === id) { return kinds[i]; } }
  return kinds[0];
}

// Параметры прогона по положению органов управления. Модельное время носит n₀: точки кадров
// параметра t не принимают, поэтому t на страницу входит только как n₀ = round(t·Fs).
function currentParams() {
  const sampleRate = sampleRateSteps[+el('fs').value];
  const single = el('jsingle').checked;
  const count = +el('jcount').value;
  const pick = +el('jpick').value;
  const amplitude = +el('amp').value;
  const phiIndex = +el('phi').value;
  const time = +el('tm').value / 10;
  const residualHz = +el('df').value * 10000;
  const parameters = {
    sampleRate: sampleRate,
    referenceFreq: carrierFreqHz - residualHz,
    residualHz: residualHz,
    startSample: Math.round(time * sampleRate),
    time: time,
    single: single,
    count: count,
    pick: pick,
    amplitude: amplitude,
    phiIndex: phiIndex,
    satellites: single ? String(pick) : ('1:' + count),
    phi: (phiIndex === 0) ? '0' : String(phiIndex * Math.PI / 16),
    amplitudes: '1'
  };
  if (amplitude !== 1) {
    if (single) {
      parameters.amplitudes = amplitudeText(amplitude);
    } else {
      const list = [];
      for (let j = 1; j <= count; ++j) { list.push((j === pick) ? amplitudeText(amplitude) : '1'); }
      parameters.amplitudes = list.join(',');
    }
  }
  return parameters;
}

// Имена параметров совпадают с ключами модуля запуска (контракт, раздел 4)
function queryOf(parameters) {
  let query = 'fs=' + parameters.sampleRate + '&f0=' + parameters.referenceFreq
            + '&n0=' + parameters.startSample + '&j=' + parameters.satellites;
  if (parameters.amplitudes !== '1') { query += '&amp=' + parameters.amplitudes; }
  if (parameters.phi !== '0') { query += '&phi0=' + parameters.phi; }
  return query;
}

function summaryOf(parameters) {
  const composition = parameters.single ? ('{' + parameters.pick + '}') : ('1…' + parameters.count);
  return 'J = ' + composition
       + ' · A' + subscript(parameters.pick) + ' = ' + numberRu(parameters.amplitude, 2)
       + ' · Fs = ' + numberRu(parameters.sampleRate / 1e6, 3) + ' МГц'
       + ' · Δf = ' + numberRu(parameters.residualHz / 1000, 0) + ' кГц'
       + ' · n₀ = ' + numberRu(parameters.startSample)
       + ' · φ₀ = ' + phiLabel(parameters.phiIndex);
}

function updateLabels() {
  const single = el('jsingle').checked;
  const count = +el('jcount').value;
  const picker = el('jpick');
  picker.max = single ? 24 : count;
  if (+picker.value > +picker.max) { picker.value = picker.max; }
  el('jcountOut').textContent = single ? 'не применяется' : ('J = 1…' + count);
  el('jpickOut').textContent = 'j = ' + picker.value;
  el('ampOut').textContent = 'A' + subscript(picker.value) + ' = ' + numberRu(+el('amp').value, 2);
  el('fsOut').textContent = numberRu(sampleRateSteps[+el('fs').value] / 1e6, 3) + ' МГц';
  el('dfOut').textContent = numberRu(+el('df').value * 10, 0) + ' кГц';
  el('tmOut').textContent = numberRu(+el('tm').value / 10, 1) + ' с';
  el('phiOut').textContent = 'φ₀ = ' + phiLabel(+el('phi').value);
}

function tile(box, key, value, kind) {
  const item = document.createElement('div');
  const keyBox = document.createElement('div');
  const valueBox = document.createElement('div');
  item.className = 'tile' + (kind ? (' ' + kind) : '');
  keyBox.className = 'k';
  keyBox.textContent = key;
  valueBox.className = 'v';
  valueBox.textContent = value;
  item.appendChild(keyBox);
  item.appendChild(valueBox);
  box.appendChild(item);
}

// Показатели — только точка режима А: они выводятся аналитически и стоят единицы миллисекунд.
// Величины прогонного происхождения (пик-фактор, граница шкалы, СКЗ квадратур) читаются с кадра.
function renderTiles(box, state, parameters) {
  box.textContent = '';
  if (!state) { return; }
  tile(box, 'Число активных НКА |J|', numberRu(state.satelliteCount));
  tile(box, 'Коэффициент нормировки η', numberRu(state.normalizationFactor, 6));
  tile(box, 'Полоса модели B_model, МГц', numberRu(state.modelBandwidthHz / 1e6, 3));
  tile(box, 'Остаточная расстройка Δf, кГц', numberRu(state.residualFreqHz / 1000, 1));
  tile(box, 'Условие представимости В.2', state.representable ? 'выполнено' : 'нарушено',
       state.representable ? '' : 'warn');
  tile(box, 'Отсчёт n₀', numberRu(state.n));
  tile(box, 'Модельное время t = n₀/Fs, с', numberRu(state.n / parameters.sampleRate, 3));
  tile(box, 'Номер строки сообщения ℓ', numberRu(state.message.lineIndex));
  tile(box, 'Тип строки', lineTypeNames[state.message.lineType] || state.message.lineType);
  tile(box, 'Символ строки w[n₀]',
       numberRu(state.message.convSymbolIndex) + ' из ' + numberRu(state.message.lineLength));
}

function signedRu(value, digits) {
  return ((value > 0) ? '+' : '') + numberRu(value, digits);
}

function renderDiff() {
  const box = el('diffMetrics');
  box.textContent = '';
  if (!referenceState || !currentState) {
    const hint = document.createElement('div');
    hint.className = 'hint';
    hint.textContent = referenceState ? 'разность не определена: показатели не получены'
                                      : 'опорная конфигурация не задана';
    box.appendChild(hint);
    return;
  }
  tile(box, 'Δ числа активных НКА', signedRu(currentState.satelliteCount - referenceState.satelliteCount));
  tile(box, 'Δ коэффициента нормировки η',
       signedRu(currentState.normalizationFactor - referenceState.normalizationFactor, 6));
  tile(box, 'Δ остаточной расстройки, кГц',
       signedRu((currentState.residualFreqHz - referenceState.residualFreqHz) / 1000, 1));
  tile(box, 'Δ отсчёта n₀', signedRu(currentState.n - referenceState.n));
}

function showReject(slot, status, message) {
  const box = document.createElement('div');
  box.className = 'reject';
  box.textContent = (status > 0) ? ('кадр отклонён, код ' + status + ': ' + message)
                                 : ('обращение не выполнено: ' + message);
  slot.textContent = '';
  slot.appendChild(box);
}

// Кадр запрашивается изображением и вставляется через <img>: каждый SVG остаётся отдельным
// документом, поэтому одинаковые внутренние идентификаторы шести кадров не сталкиваются.
async function loadFrame(slot, kind, query, token) {
  slot.classList.add('busy');
  try {
    const response = await fetch('/v1/frames/' + kind.id + '.svg?' + query);
    if (token !== frameToken) { return; }
    if (!response.ok) {
      const text = await response.text();
      let message = text;
      try { message = JSON.parse(text).message; } catch (error) { message = text; }
      showReject(slot, response.status, message);
      return;
    }
    const blob = await response.blob();
    if (token !== frameToken) { return; }
    const objectUrl = URL.createObjectURL(blob);
    const image = new Image();
    image.alt = kind.alt;
    image.onload = function () { URL.revokeObjectURL(objectUrl); };
    image.src = objectUrl;
    slot.textContent = '';
    slot.appendChild(image);
  } catch (error) {
    if (token === frameToken) { showReject(slot, 0, String(error)); }
  } finally {
    slot.classList.remove('busy');
  }
}

// Кадры перестраиваются по отпусканию органа управления: комплект шести кадров при полном
// составе стоит около 146 мс, и при перерисовке на каждое движение обращения копятся быстрее,
// чем разбираются (ввод-вывод блокирующий, контракт п. 10.4).
function refreshFrames() {
  const query = queryOf(currentParams());
  const token = ++frameToken;
  if (layout === 'overview') {
    kinds.forEach(function (kind) { loadFrame(el('slot-' + kind.id), kind, query, token); });
    return;
  }
  const kind = kindOf(selectedKind);
  if (layout === 'detail') {
    loadFrame(el('slotOne'), kind, query, token);
    return;
  }
  loadFrame(el('slotCur'), kind, query, token);
  if (referenceQuery) { loadFrame(el('slotRef'), kind, referenceQuery, token); }
}

function renderStateError(status, message) {
  currentState = null;
  [el('metrics'), el('curMetrics')].forEach(function (box) {
    box.textContent = '';
    tile(box, 'Показатели не получены, код ' + status, message, 'wide');
  });
  renderDiff();
}

// Показатели перестраиваются на каждое движение органа управления с подавлением дребезга
async function refreshState() {
  const parameters = currentParams();
  el('curParams').textContent = summaryOf(parameters);
  try {
    const response = await fetch('/v1/state?' + queryOf(parameters));
    const text = await response.text();
    if (!response.ok) {
      let message = text;
      try { message = JSON.parse(text).message; } catch (error) { message = text; }
      renderStateError(response.status, message);
      return;
    }
    currentState = JSON.parse(text);
    renderTiles(el('metrics'), currentState, parameters);
    renderTiles(el('curMetrics'), currentState, parameters);
    renderDiff();
  } catch (error) {
    renderStateError(0, String(error));
  }
}

function scheduleState() {
  clearTimeout(stateTimer);
  stateTimer = setTimeout(refreshState, 150);
}

async function loadInfo() {
  try {
    const response = await fetch('/v1/info');
    const info = await response.json();
    el('origin').textContent = info.service + ' ' + info.version + ' · интерфейс ' + info.api
                             + ' · тракт ' + info.band + ' · профиль ' + info.icdProfile;
  } catch (error) {
    el('origin').textContent = 'сведения о сервисе недоступны';
  }
}

function selectKind(id) {
  selectedKind = id;
  const buttons = el('tabs').children;
  for (let i = 0; i < buttons.length; ++i) {
    buttons[i].classList.toggle('on', buttons[i].dataset.kind === id);
  }
  if (layout !== 'overview') { refreshFrames(); }
}

function setLayout(name) {
  layout = name;
  document.body.className = 'layout-' + name;
  const buttons = el('layouts').children;
  for (let i = 0; i < buttons.length; ++i) {
    buttons[i].classList.toggle('on', buttons[i].dataset.layout === name);
  }
  refreshFrames();
}

function applyReference() {
  const parameters = currentParams();
  referenceParams = parameters;
  referenceQuery = queryOf(parameters);
  referenceState = currentState;
  el('refParams').textContent = summaryOf(parameters);
  renderTiles(el('refMetrics'), referenceState, parameters);
  renderDiff();
  refreshFrames();
}

function resetControls() {
  el('jcount').value = 24;
  el('jsingle').checked = false;
  el('jpick').value = 5;
  el('amp').value = 1;
  el('fs').value = Math.max(0, sampleRateSteps.indexOf(defaultSampleRate));
  el('df').value = 0;
  el('tm').value = 0;
  el('phi').value = 0;
  updateLabels();
  refreshState();
  refreshFrames();
}

function init() {
  const tabs = el('tabs');
  const grid = el('grid');
  kinds.forEach(function (kind) {
    const button = document.createElement('button');
    button.type = 'button';
    button.textContent = kind.tab;
    button.title = kind.alt;
    button.dataset.kind = kind.id;
    button.className = (kind.id === selectedKind) ? 'on' : '';
    button.onclick = function () { selectKind(kind.id); };
    tabs.appendChild(button);

    const slot = document.createElement('div');
    slot.className = 'slot';
    slot.id = 'slot-' + kind.id;
    slot.title = 'открыть в разборе: ' + kind.alt;
    slot.onclick = function () { selectKind(kind.id); setLayout('detail'); };
    grid.appendChild(slot);
  });

  el('fs').max = sampleRateSteps.length - 1;
  el('fs').value = Math.max(0, sampleRateSteps.indexOf(defaultSampleRate));

  ['jcount', 'jsingle', 'jpick', 'amp', 'fs', 'df', 'tm', 'phi'].forEach(function (id) {
    const control = el(id);
    control.addEventListener('input', function () { updateLabels(); scheduleState(); });
    control.addEventListener('change', function () { updateLabels(); refreshFrames(); });
  });

  const layoutButtons = el('layouts').children;
  for (let i = 0; i < layoutButtons.length; ++i) {
    layoutButtons[i].onclick = function () { setLayout(this.dataset.layout); };
  }
  el('pin').onclick = applyReference;
  el('reset').onclick = resetControls;

  updateLabels();
  renderDiff();
  loadInfo();
  refreshState();
  refreshFrames();
}

init();
</script>
</body>
</html>
)PANEL";

// Числовые параметры сигнала в тексте страницы не дублируются: опорная частота и нижняя
// граница ряда Fs (= 2·B_model — граница условия представимости В.2 при Δf = 0) подставляются
// из glonass/types.h, значение по умолчанию — из ключей модуля запуска.
std::string composePage() {
   std::string page = pageHead;

   page += "const carrierFreqHz = " + std::to_string(glonass::carrierFreqL1OC) + ";\n";
   page += "const sampleRateSteps = [" + std::to_string(2 * glonass::modelBandwidthL1OC)
           + ", 5000000, 10000000, " + std::to_string(glonass_params::defaultSampleRate)
           + ", 40000000];\n";
   page += "const defaultSampleRate = " + std::to_string(glonass_params::defaultSampleRate) + ";\n";
   page += pageTail;
   return page;
}
} // namespace

const std::string &panelPageHtml() {
   static const std::string page = composePage();

   return page;
}
} // namespace glonass_service
