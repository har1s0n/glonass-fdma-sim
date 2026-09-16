#include "panel_receiver_views.h"

namespace glonass_service {
namespace {
// Сценарий видов компоновки «Приёмник» на темах corr, corr_hist, log, psd, sat_stat, pvt_sol.
constexpr const char* receiverViewsScript =
   R"PANEL(

// Окно дополнительных корреляторов, с: умолчание приёмника DEF_SEL_WIDTH (src/sdr_web.c);
// 101 коррелятор занимает ±width/2 вокруг точного коррелятора P (sdr_ch_set_corr, src/sdr_ch.c)
const corrWindowSeconds = 3e-6;
// История точного коррелятора для плоскости IP–QP и рядов IP, QP, с
const corrHistorySeconds = 1;
// Оценка СПМ приёмником: точек и время усреднения, с (умолчания интерфейса PocketSDR, rfch.js)
const psdPoints = 2048;
const psdAverageSeconds = 0.01;
// Строк НС в таблице вида
const navRowsKept = 60;
const corrStateNames = { 1: 'простой', 2: 'поиск', 3: 'сопровождение' };
const corrStateLock = 3; // SDR_STATE_LOCK (src/pocket_sdr.h)

let corrChosenPrn = 0;         // НКА, выбранный на панели; 0: по умолчанию
let corrSubscribedChannel = 0; // канал приёмника, на который оформлена подписка corr
let corrSnapshot = null;
let corrHistory = null;
let psdFrame = null;
let navRecords = [];           // строки $NAV текущего прогона приёмника, новые первыми
let navLastTime = -Infinity;
let navModel = null;           // { sessionId, bits }: n_с из кадра navline сеанса
let skyStatus = null;
let solutionStatus = null;

receiverViews.push(
  { id: 'corr', tab: 'Корреляторы', alt: 'Корреляторы выбранного канала приёмника',
    block: 'Внешний приёмник PocketSDR',
    what: 'Корреляторы выбранного канала: форма корреляционной функции по 101 дополнительному коррелятору в окне 3 мкс вокруг точного коррелятора P, точки IP и QP на плоскости и их ряды за последнюю секунду.',
    why: 'по форме пика и положению раннего и позднего корреляторов E, L видно, держит ли петля задержки максимум; по плоскости IP–QP виден захват фазы.',
    look: 'у истинного канала пик симметричен и E, L стоят на равной высоте; при захвате фазы точки лежат на оси IP. Ось задержки в чипах уплотнения 1/f_T1; окно и расстановка корреляторов заданы приёмником.',
    topics: ['corr', 'corr_hist'], subscribe: corrSubscribe, render: renderCorrView, bar: renderCorrBar },
  { id: 'nav', tab: 'Строки НС', alt: 'Строки навигационного сообщения, декодированные приёмником',
    block: 'Внешний приёмник PocketSDR',
    what: 'Строки навигационного сообщения L1OCd, декодированные приёмником: время приёмника, НКА, длина строки, число исправленных ошибок и сама строка в шестнадцатеричной записи.',
    why: 'запись появляется только при сошедшемся циклическом коде, поэтому она служит независимой проверкой блока Б_L1OC внешней реализацией.',
    look: 'длина строки у приёмника должна совпадать с n_с модели из кадра строки того же сеанса; исправленных ошибок без шума нет. Цифровая информация модели нулевая, поэтому строки всех НКА одинаковы; побитовая сверка появится вместе с блоком генерации навигационного сообщения.',
    topics: ['log'], subscribe: navSubscribe, render: renderNavView },
  { id: 'psd', tab: 'Спектр входа', alt: 'Спектральная плотность мощности на входе приёмника',
    block: 'Внешний приёмник PocketSDR',
    what: 'Спектральная плотность мощности потока на входе приёмника; оценку делает сам приёмник по отсчётам CS16.',
    why: 'показывает, что получает приёмник после квантования: занятую полосу и положение спектра.',
    look: 'ось отсчитывается от частоты гетеродина приёмника; расстройка Δf смещает спектр на Δf. Разметки модели на виде нет: спектр того же сигнала на стороне модели показывает вкладка СПМ компоновок модели.',
    topics: ['psd'], subscribe: psdSubscribe, render: renderPsdView },
  { id: 'sky', tab: 'Небо', alt: 'Положения НКА на небосводе',
    block: 'Внешний приёмник PocketSDR',
    what: 'Положения НКА на небосводе: азимут по кругу от севера, угол места от горизонта на краю к зениту в центре.',
    why: 'азимут и угол места приёмник вычисляет по эфемеридам из навигационного сообщения и по своему решению.',
    look: 'цифровая информация модели нулевая: эфемерид и решения нет, поэтому вид пуст; он будет отлажен вместе с блоком генерации навигационного сообщения.',
    topics: ['sat_stat'], subscribe: skySubscribe, render: renderSkyView },
  { id: 'sol', tab: 'Решение', alt: 'Навигационное решение приёмника',
    block: 'Внешний приёмник PocketSDR',
    what: 'Навигационное решение приёмника: время, широта, долгота, высота и число НКА в решении.',
    why: 'итог работы всего тракта приёмника.',
    look: 'решению нужны эфемериды; цифровая информация модели нулевая, решения нет. Вид будет отлажен вместе с блоком генерации навигационного сообщения.',
    topics: ['pvt_sol'], subscribe: solutionSubscribe, render: renderSolutionView }
);

// Подписка на темы активного вида; отписка по именам тем
function receiverViewSubscribe() {
  const view = receiverViewOf(receiverView);
  if (view.subscribe) { view.subscribe(); }
}

function receiverViewUnsubscribe() {
  const view = receiverViewOf(receiverView);
  (view.topics || []).forEach(function (topic) { receiverSend({ cmd: 'unsub', topic: topic }); });
  corrSubscribedChannel = 0;
}

function receiverViewsReset() {
  corrSnapshot = null;
  corrHistory = null;
  psdFrame = null;
  navRecords = [];
  navLastTime = -Infinity;
  skyStatus = null;
  solutionStatus = null;
}

function corrSubscribe() {
  const channel = corrTarget();
  corrSubscribedChannel = channel ? channel.ch : 0;
  corrSnapshot = null;
  corrHistory = null;
  if (!channel) { return; }
  receiverSend({ cmd: 'sub', topic: 'corr', ch: channel.ch, cyc: 100, width: corrWindowSeconds });
  receiverSend({ cmd: 'sub', topic: 'corr_hist', ch: channel.ch, cyc: 100, tspan: corrHistorySeconds });
}

// Журнал отдаётся новому подписчику с последних 2000 строк, поэтому таблица строится заново при
// каждой подписке
function navSubscribe() {
  navRecords = [];
  navLastTime = -Infinity;
  receiverSend({ cmd: 'sub', topic: 'log', cyc: 200 });
}

function psdSubscribe() {
  psdFrame = null;
  receiverSend({ cmd: 'sub', topic: 'psd', rfch: 1, cyc: 200, tave: psdAverageSeconds, nfft: psdPoints });
}

// Обозначения НКА у приёмника: R и двузначный номер, как в таблице каналов (SAT R01…R24)
function skySatellites() {
  const list = [];
  for (let prn = 1; prn <= +el('jcount').max; ++prn) { list.push('R' + ((prn < 10) ? '0' : '') + prn); }
  return list;
}

function skySubscribe() {
  receiverSend({ cmd: 'sub', topic: 'sat_stat', cyc: 1000, sats: skySatellites().join(',') });
}

function solutionSubscribe() {
  receiverSend({ cmd: 'sub', topic: 'pvt_sol', cyc: 1000 });
}

// Канал корреляторов: выбранный на панели НКА, иначе первый истинный, иначе первый НКА состава,
// иначе первый сопровождаемый
function corrTarget() {
  if (!receiverChannels) { return null; }
  const rows = channelRows().filter(function (item) { return item.cls !== 'missing'; });
  const members = receiverSession ? receiverSession.members : [];
  const pick = function (test) {
    for (let i = 0; i < rows.length; ++i) {
      if (test(rows[i])) { return rows[i]; }
    }
    return null;
  };
  const chosen = (corrChosenPrn ? pick(function (item) { return item.row.prn === corrChosenPrn; }) : null)
              || pick(function (item) { return item.cls === 'genuine'; })
              || pick(function (item) { return members.indexOf(item.row.prn) >= 0; })
              || rows[0] || null;
  return chosen ? { prn: chosen.row.prn, ch: chosen.row.ch, cls: chosen.cls } : null;
}

// Вызывается после каждой таблицы каналов: подписка корреляторов следует за выбранным каналом
function receiverViewChannelsChanged() {
  if (!receiverActive || (receiverView !== 'corr')) { return; }
  const channel = corrTarget();
  if ((channel ? channel.ch : 0) !== corrSubscribedChannel) { corrSubscribe(); }
  renderReceiverBar();
}

function renderReceiverBar() {
  const bar = el('rxBar');
  const view = receiverViewOf(receiverView);
  if (!receiverActive || !view.bar) {
    bar.textContent = '';
    return;
  }
  view.bar(bar);
}

// Выбор НКА для корреляторов. Элемент списка создаётся один раз: пересоздание при каждой смене
// сопровождаемых каналов уничтожало открытый список, и выбор терялся. Пункты обновляются на месте;
// пока список в фокусе, они не трогаются, и отложенное обновление применяется при потере фокуса
function renderCorrBar(bar) {
  let select = bar.querySelector('select');
  if (!select) {
    const label = document.createElement('label');
    bar.textContent = '';
    label.textContent = 'НКА для корреляторов';
    label.htmlFor = 'rxCorrPrn';
    select = document.createElement('select');
    select.id = 'rxCorrPrn';
    select.onchange = function () {
      corrChosenPrn = +select.value;
      corrSubscribe();
      renderReceiverBar();
      requestReceiverRender();
    };
    select.onblur = renderReceiverBar;
    bar.appendChild(label);
    bar.appendChild(select);
  }
  if (document.activeElement === select) { return; }
  const rows = receiverChannels ? channelRows().filter(function (item) { return item.cls !== 'missing'; }) : [];
  const items = [{ value: '0', text: 'по умолчанию: первый истинный' }].concat(rows.map(function (item) {
    return { value: String(item.row.prn), text: 'НКА ' + item.row.prn + ' · ' + classNames[item.cls] };
  }));
  const key = items.map(function (item) { return item.value + ':' + item.text; }).join('|') + '|' + corrChosenPrn;
  if (select.dataset.key === key) { return; }
  select.dataset.key = key;
  items.forEach(function (item, index) {
    const option = select.options[index] || select.appendChild(document.createElement('option'));
    option.value = item.value;
    option.textContent = item.text;
  });
  while (select.options.length > items.length) { select.remove(select.options.length - 1); }
  select.value = String(corrChosenPrn);
  if (select.value !== String(corrChosenPrn)) { select.value = '0'; } // выбранный НКА сейчас не сопровождается
}

// Двоичный кадр (протокол v1, Binary frames): байт 0 задаёт тип
function decodeReceiverFrame(buffer) {
  const view = new DataView(buffer);
  const type = view.getUint8(0);
  if (type === 1) {
    const n = view.getUint32(12, true);
    return { type: 'psd', rfch: view.getUint8(1), iq: view.getUint8(2), fs: view.getFloat32(4, true),
             tave: view.getFloat32(8, true), n: n, fo: view.getFloat64(16, true),
             psd: new Float32Array(buffer, 24, n) };
  }
  if (type === 2) {
    const n = view.getUint32(4, true);
    let offset = 40;
    const pos = new Float32Array(buffer, offset, n);
    offset += 12 * n; // мгновенные I, Q корреляторов в виде не используются
    const aveP = new Float32Array(buffer, offset, n);
    offset += 4 * n;
    const aveI = new Float32Array(buffer, offset, n);
    return { type: 'corr', state: view.getUint8(1), ch: view.getUint16(2, true), n: n,
             npos: view.getUint32(8, true), fs: view.getFloat32(12, true), lock: view.getFloat32(16, true),
             cn0: view.getFloat32(20, true), fd: view.getFloat32(24, true), coff: view.getFloat64(32, true),
             pos: pos, aveP: aveP, aveI: aveI };
  }
  if (type === 3) {
    const n = view.getUint32(4, true);
    return { type: 'corr_hist', ch: view.getUint16(2, true), n: n, period: view.getFloat32(16, true),
             points: new Float32Array(buffer, 24, 2 * n) };
  }
  return null;
}

function receiverBinaryMessage(buffer) {
  const frame = decodeReceiverFrame(buffer);
  if (!frame) { return false; }
  if (frame.type === 'psd') {
    psdFrame = frame;
  } else if (frame.type === 'corr') {
    corrSnapshot = frame;
  } else {
    corrHistory = frame;
  }
  return receiverActive && ((receiverView === 'corr') || (receiverView === 'psd'));
}

// Прочие сообщения JSON; sel_ch виду не нужен: после снятия интерфейса PocketSDR панель остаётся
// единственным клиентом приёмника
function receiverExtraMessage(message) {
  if (message.type === 'log') { return navCollect(message.lines) && (receiverView === 'nav'); }
  if (message.type === 'sat_stat') {
    skyStatus = Array.isArray(message.sats) ? message.sats : [];
    return receiverView === 'sky';
  }
  if (message.type === 'pvt_sol') {
    solutionStatus = parseSolution(String(message.str || ''));
    return receiverView === 'sol';
  }
  return false;
}

// Запись $NAV (формат out_log_nav, src/sdr_nav.c): время приёмника, с; НКА; сигнал; PRN; число
// исправленных ошибок; длина строки, бит; строка в шестнадцатеричной записи. Журнал хранит и
// прошлые прогоны: убывание времени приёмника больше чем на 1 с отмечает его перезапуск, и записи
// до перезапуска отбрасываются
function navCollect(lines) {
  if (!Array.isArray(lines)) { return false; }
  let changed = false;
  lines.forEach(function (line) {
    const text = String(line);
    if (text.indexOf('$NAV,') !== 0) { return; }
    const field = text.trim().split(',');
    if ((field.length < 8) || (field[3] !== receiverSignal)) { return; }
    const time = parseFloat(field[1]);
    if (time < navLastTime - 1) {
      navRecords = [];
      navLastTime = time;
    } else {
      navLastTime = Math.max(navLastTime, time);
    }
    navRecords.unshift({ time: time, sat: field[2], prn: +field[4], nerr: +field[5], bits: +field[6],
                         hex: field[7] });
    changed = true;
  });
  if (navRecords.length > navRowsKept) { navRecords.length = navRowsKept; }
  return changed;
}

// Длина строки модели n_с (Б_L1OC.3) из кадра navline того же сеанса; запрашивается раз на сеанс
async function navModelBits() {
  const session = receiverSession;
  if (!session || (navModel && (navModel.sessionId === session.id))) { return; }
  navModel = { sessionId: session.id, bits: null };
  try {
    const response = await fetch('/v1/frames/navline?' + session.query);
    if (!response.ok) { return; }
    const frame = await response.json();
    navModel.bits = frame.line.lineBits;
    requestReceiverRender();
  } catch (error) {
    // кадр модели недоступен: сверка длины не выполняется
  }
}

// Строка решения (формат sdr_pvt_solstr, src/sdr_pvt.c): дата и время; широта и долгота, градусы;
// высота, м; НКА в решении из отслеживаемых; признак решения FIX
function parseSolution(text) {
  const field = text.trim().split(/\s+/);
  if (field.length < 7) { return null; }
  return { time: field[0] + ' ' + field[1], lat: parseFloat(field[2]), lon: parseFloat(field[3]),
           height: parseFloat(field[4]), used: field[5], fixed: field[6] === 'FIX' };
}

function prnClass(prn) {
  const rows = channelRows();
  for (let i = 0; i < rows.length; ++i) {
    if (rows[i].row.prn === prn) { return rows[i].cls; }
  }
  return 'unknown';
}

function niceStep(value) {
  const power = Math.pow(10, Math.floor(Math.log10(value)));
  const mantissa = value / power;
  return ((mantissa <= 1) ? 1 : (mantissa <= 2) ? 2 : (mantissa <= 5) ? 5 : 10) * power;
}

// Знаков после запятой в подписях шкалы с данным шагом
function digitsFor(step) {
  return Math.max(0, -Math.floor(Math.log10(step) + 1e-9));
}

function svgLine(x1, y1, x2, y2, style) {
  return '<line x1="' + x1.toFixed(1) + '" y1="' + y1.toFixed(1) + '" x2="' + x2.toFixed(1) + '" y2="'
       + y2.toFixed(1) + '" style="' + style + '"/>';
}

// Поле графика: сетка по шагам, подписи осей; возвращает функции перевода значений в координаты
function svgPlotField(field, xLow, xHigh, xStep, yLow, yHigh, yStep) {
  const xDigits = digitsFor(xStep);
  const yDigits = digitsFor(yStep);
  const xOf = function (value) { return field.left + (value - xLow) / (xHigh - xLow) * field.width; };
  const yOf = function (value) { return field.top + (1 - (value - yLow) / (yHigh - yLow)) * field.height; };
  let content = '<rect x="' + field.left + '" y="' + field.top + '" width="' + field.width + '" height="'
              + field.height + '" style="fill:none;stroke:var(--line)"/>';
  for (let value = Math.ceil(xLow / xStep) * xStep; value <= xHigh + 1e-9; value += xStep) {
    content += svgLine(xOf(value), field.top, xOf(value), field.top + field.height,
                       (Math.abs(value) < 1e-9) ? 'stroke:var(--muted)' : 'stroke:var(--line)');
    content += svgText(xOf(value), field.top + field.height + 15, numberRu(value, xDigits),
                       'font-size="10.5" style="fill:var(--muted)" text-anchor="middle"');
  }
  for (let value = Math.ceil(yLow / yStep) * yStep; value <= yHigh + 1e-9; value += yStep) {
    content += svgLine(field.left, yOf(value), field.left + field.width, yOf(value),
                       (Math.abs(value) < 1e-9) ? 'stroke:var(--muted)' : 'stroke:var(--line)');
    content += svgText(field.left - 6, yOf(value) + 4, numberRu(value, yDigits),
                       'font-size="10.5" style="fill:var(--muted)" text-anchor="end"');
  }
  return { content: content, xOf: xOf, yOf: yOf };
}

function svgAxisTitles(field, xTitle, yTitle) {
  const middle = field.top + field.height / 2;
  return svgText(field.left + field.width / 2, field.top + field.height + 33, xTitle,
                 'font-size="11.5" style="fill:var(--muted)" text-anchor="middle"')
       + svgText(field.left - 46, middle, yTitle, 'font-size="11.5" style="fill:var(--muted)" text-anchor="middle" '
                 + 'transform="rotate(-90 ' + (field.left - 46).toFixed(1) + ' ' + middle.toFixed(1) + ')"');
}

function clampValue(value, low, high) {
  return Math.max(low, Math.min(high, value));
}

function renderCorrView(box) {
  if (!receiverOpen()) {
    showHint(box, 'связь с приёмником не установлена');
    return;
  }
  const channel = corrTarget();
  if (!channel) {
    showHint(box, 'приёмник не сопровождает ни одного канала');
    return;
  }
  const snapshot = (corrSnapshot && (corrSnapshot.ch === channel.ch)) ? corrSnapshot : null;
  const history = (corrHistory && (corrHistory.ch === channel.ch)) ? corrHistory : null;
  if (!snapshot) {
    showHint(box, 'корреляторы канала НКА ' + channel.prn + ' ожидаются от приёмника');
    return;
  }
  const width = 960;
  const height = 610;
  // Задержка корреляторов в чипах уплотнения: отсчёты приёмника / Fs · f_T1
  const chipsOf = function (samples) { return samples / snapshot.fs * chipRateL1OC; };
  const norm = (snapshot.aveI[0] > 0) ? snapshot.aveI[0]
             : Math.max.apply(null, Array.prototype.map.call(snapshot.aveI, Math.abs)) || 1;
  const extra = [];
  let yLowest = -0.2;
  let yHighest = 1.2;
  for (let i = snapshot.npos; i < snapshot.n; ++i) {
    extra.push([chipsOf(snapshot.pos[i]), snapshot.aveI[i] / norm]);
    yLowest = Math.min(yLowest, snapshot.aveI[i] / norm);
    yHighest = Math.max(yHighest, snapshot.aveI[i] / norm);
  }
  extra.sort(function (a, b) { return a[0] - b[0]; });
  const xLimit = corrWindowSeconds / 2 * chipRateL1OC * 1.05;
  const shape = { left: 70, top: 76, width: 540, height: 260 };
  const plane = { left: 690, top: 76, width: 250, height: 250 };
  const series = { left: 70, top: 420, width: 870, height: 135 };
  const fieldA = svgPlotField(shape, -xLimit, xLimit, 0.5, Math.floor(yLowest * 2) / 2, Math.ceil(yHighest * 2) / 2,
                              0.5);
  let content = fieldA.content;
  if (extra.length) {
    content += '<path d="' + extra.map(function (point, index) {
      return ((index === 0) ? 'M' : 'L') + fieldA.xOf(point[0]).toFixed(1) + ' ' + fieldA.yOf(point[1]).toFixed(1);
    }).join(' ') + '" style="fill:none;stroke:var(--accent);stroke-width:1.6"/>';
  }
  // Основные корреляторы: P, E, L (src/sdr_ch.c, trk_new); коррелятор шума N лежит вне окна
  ['P', 'E', 'L'].forEach(function (name, index) {
    if (index >= snapshot.npos) { return; }
    const x = clampValue(chipsOf(snapshot.pos[index]), -xLimit, xLimit);
    const y = snapshot.aveI[index] / norm;
    content += '<circle cx="' + fieldA.xOf(x).toFixed(1) + '" cy="' + fieldA.yOf(y).toFixed(1)
             + '" r="4.5" style="fill:var(--mark)"/>';
    content += svgText(fieldA.xOf(x), fieldA.yOf(y) - 9, name, 'font-size="11" font-weight="700" '
                       + 'style="fill:var(--mark)" text-anchor="middle"');
  });
  content += svgAxisTitles(shape, 'задержка относительно P, чипов уплотнения', 'I·sign(IP), доля P');
  if (history && history.n) {
    let amplitude = 0;
    for (let i = 0; i < 2 * history.n; ++i) { amplitude = Math.max(amplitude, Math.abs(history.points[i])); }
    amplitude = amplitude || 1;
    const scale = amplitude * 1.1;
    const step = niceStep(scale / 2);
    const fieldB = svgPlotField(plane, -scale, scale, step, -scale, scale, step);
    let dots = '';
    for (let i = 0; i < history.n; ++i) {
      dots += 'M' + fieldB.xOf(history.points[2 * i]).toFixed(1) + ' ' + fieldB.yOf(history.points[2 * i + 1]).toFixed(1)
            + 'h0.01';
    }
    content += fieldB.content + '<path d="' + dots + '" style="fill:none;stroke:var(--accent);stroke-width:3;'
             + 'stroke-linecap:round"/>';
    content += svgText(plane.left + plane.width / 2, plane.top + plane.height + 33, 'IP',
                       'font-size="11.5" style="fill:var(--muted)" text-anchor="middle"');
    content += svgText(plane.left - 40, plane.top + plane.height / 2, 'QP', 'font-size="11.5" style="fill:var(--muted)" '
                       + 'text-anchor="middle"');
    const span = (history.n - 1) * history.period;
    const fieldC = svgPlotField(series, -span, 0, niceStep(span / 8), -scale, scale, step);
    let inPhase = '';
    let quadrature = '';
    for (let i = 0; i < history.n; ++i) {
      const x = fieldC.xOf(-span + i * history.period).toFixed(1);
      inPhase += ((i === 0) ? 'M' : 'L') + x + ' ' + fieldC.yOf(history.points[2 * i]).toFixed(1);
      quadrature += ((i === 0) ? 'M' : 'L') + x + ' ' + fieldC.yOf(history.points[2 * i + 1]).toFixed(1);
    }
    content += fieldC.content + '<path d="' + quadrature + '" style="fill:none;stroke:var(--mark);stroke-width:1.2"/>'
             + '<path d="' + inPhase + '" style="fill:none;stroke:var(--accent);stroke-width:1.2"/>';
    content += svgAxisTitles(series, 'время относительно последней точки, с', 'IP, QP');
    content += '<rect x="' + (series.left + series.width - 120) + '" y="' + (series.top - 17)
             + '" width="10" height="10" rx="2" style="fill:var(--accent)"/>'
             + svgText(series.left + series.width - 106, series.top - 8, 'IP', 'font-size="11" style="fill:var(--fg)"')
             + '<rect x="' + (series.left + series.width - 70) + '" y="' + (series.top - 17)
             + '" width="10" height="10" rx="2" style="fill:var(--mark)"/>'
             + svgText(series.left + series.width - 56, series.top - 8, 'QP', 'font-size="11" style="fill:var(--fg)"');
  } else {
    content += svgText(plane.left + plane.width / 2, plane.top + plane.height / 2, 'история IP, QP ожидается',
                       'font-size="12" style="fill:var(--muted)" text-anchor="middle"');
  }
  // В сопровождении к названию состояния добавляется его длительность, иначе выводится одно название
  const state = (snapshot.state === corrStateLock) ? ('сопровождение ' + numberRu(snapshot.lock, 1) + ' с')
              : (corrStateNames[snapshot.state] || 'состояние неизвестно');
  const subtitle = classNames[channel.cls] + ' · ' + state + ' · C/N0 ' + numberRu(snapshot.cn0, 1)
                 + ' дБ·Гц · DOP ' + signedRu(snapshot.fd, 1) + ' Гц · COFF ' + numberRu(snapshot.coff, 5) + ' мс';
  box.innerHTML = svgFrame(width, height, 'Корреляторы канала НКА ' + channel.prn, subtitle, content);
}

function renderNavView(box) {
  navModelBits();
  if (!receiverOpen()) {
    showHint(box, 'связь с приёмником не установлена');
    return;
  }
  const session = receiverSession;
  const modelBits = (session && navModel && (navModel.sessionId === session.id)) ? navModel.bits : null;
  const matching = navRecords.filter(function (record) { return record.bits === modelBits; }).length;
  const corrected = navRecords.filter(function (record) { return record.nerr > 0; }).length;
  const stack = document.createElement('div');
  const tiles = document.createElement('div');
  stack.className = 'rxstack';
  tiles.className = 'tiles';
  tile(tiles, 'Строк в таблице', numberRu(navRecords.length));
  tile(tiles, 'Длина строки модели n_с, бит', (modelBits !== null) ? numberRu(modelBits)
       : (session ? 'нет данных' : 'сеанс не открыт'));
  tile(tiles, 'Длина совпадает с моделью', (modelBits !== null)
       ? (numberRu(matching) + ' из ' + numberRu(navRecords.length)) : 'нет данных',
       ((modelBits !== null) && (matching < navRecords.length)) ? 'warn' : '');
  tile(tiles, 'С исправленными ошибками', numberRu(corrected), corrected ? 'warn' : '');
  stack.appendChild(tiles);
  if (!navRecords.length) {
    const hint = document.createElement('div');
    hint.className = 'hint';
    hint.textContent = 'строки ещё не декодированы';
    stack.appendChild(hint);
    box.textContent = '';
    box.appendChild(stack);
    return;
  }
  const wrap = document.createElement('div');
  const table = document.createElement('table');
  const headRow = table.createTHead().insertRow();
  const body = table.createTBody();
  wrap.className = 'rxwrap';
  table.className = 'rxtable';
  ['Время приёмника, с', 'НКА', 'Состав J', 'Длина, бит', 'Сверка длины', 'Исправлено ошибок', 'Строка (hex)']
    .forEach(function (text, index) {
      const cell = document.createElement('th');
      cell.textContent = text;
      if ((index === 1) || (index === 2) || (index >= 4)) { cell.className = 'l'; }
      headRow.appendChild(cell);
    });
  navRecords.forEach(function (record) {
    const line = body.insertRow();
    const matches = (modelBits === null) ? '' : ((record.bits === modelBits) ? 'совпадает' : 'не совпадает');
    const cells = [
      numberRu(record.time, 3),
      String(record.prn),
      session ? ((session.members.indexOf(record.prn) >= 0) ? 'да' : 'нет') : '',
      numberRu(record.bits),
      matches,
      numberRu(record.nerr),
      record.hex
    ];
    cells.forEach(function (text, index) {
      const cell = line.insertCell();
      cell.textContent = text;
      if (index === 4) {
        cell.className = 'l ' + ((record.bits === modelBits) ? 'cls-genuine' : 'cls-spurious');
      } else if (index === 6) {
        cell.className = 'l rxmono';
      } else if ((index === 1) || (index === 2)) {
        cell.className = 'l';
      }
    });
  });
  wrap.appendChild(table);
  stack.appendChild(wrap);
  box.textContent = '';
  box.appendChild(stack);
}

function renderPsdView(box) {
  const frame = psdFrame;
  if (!frame) {
    showHint(box, receiverOpen() ? 'спектр ожидается от приёмника' : 'связь с приёмником не установлена');
    return;
  }
  const width = 960;
  const height = 390;
  const field = { left: 70, top: 64, width: 866, height: 270 };
  // Бины: при IQ ось [fo − Fs/2, fo + Fs/2) с нулём частоты посередине, при I ось [fo, fo + Fs/2)
  const low = (frame.iq === 2) ? -frame.fs / 2 : 0;
  const span = (frame.iq === 2) ? frame.fs : frame.fs / 2;
  let highest = -Infinity;
  for (let i = 0; i < frame.n; ++i) {
    if (isFinite(frame.psd[i])) { highest = Math.max(highest, frame.psd[i]); }
  }
  if (!isFinite(highest)) {
    showHint(box, 'спектр пуст: отсчёты на вход приёмника не поступают');
    return;
  }
  const yHigh = Math.ceil((highest + 3) / 10) * 10;
  const yLow = yHigh - 60;
  const lowMHz = low / 1e6;
  const highMHz = (low + span) / 1e6;
  const plot = svgPlotField(field, lowMHz, highMHz, niceStep((highMHz - lowMHz) / 10), yLow, yHigh, 10);
  let path = '';
  for (let i = 0; i < frame.n; ++i) {
    const value = isFinite(frame.psd[i]) ? clampValue(frame.psd[i], yLow, yHigh) : yLow;
    path += ((i === 0) ? 'M' : 'L') + plot.xOf((low + i * span / frame.n) / 1e6).toFixed(1) + ' '
          + plot.yOf(value).toFixed(1);
  }
  const content = plot.content + '<path d="' + path + '" style="fill:none;stroke:var(--accent);stroke-width:1.2"/>'
                + svgAxisTitles(field, 'частота относительно гетеродина fo, МГц', 'СПМ, дБ/Гц');
  const subtitle = 'оценка приёмника: ' + numberRu(frame.n) + ' точек, усреднение ' + numberRu(frame.tave * 1000)
                 + ' мс · гетеродин fo = ' + numberRu(frame.fo / 1e6, 3) + ' МГц · Fs = ' + numberRu(frame.fs / 1e6, 3)
                 + ' МГц';
  box.innerHTML = svgFrame(width, height, 'Спектральная плотность мощности на входе приёмника', subtitle, content);
}

function renderSkyView(box) {
  if (!receiverOpen()) {
    showHint(box, 'связь с приёмником не установлена');
    return;
  }
  if (!skyStatus) {
    showHint(box, 'положения НКА ожидаются от приёмника');
    return;
  }
  const width = 960;
  const height = 450;
  const cx = 260;
  const cy = 245;
  const radius = 170;
  // Положение считается вычисленным при угле места больше нуля, как в интерфейсе PocketSDR (rcv.js,
  // «suppress invalid satellites»). Признак eph без решения недостоверен: при нулевых времени
  // решения и эпохе эфемерид условие timediff(sol->time, geph->toe) <= 1800 выполняется
  // (sdr_rcv_sat_stat, src/sdr_rcv.c), а азимут и угол места остаются нулевыми
  const placed = skyStatus.filter(function (sat) {
    return isFinite(sat.az) && isFinite(sat.el) && (sat.el > 0);
  });
  const radiusOf = function (elevation) { return radius * (90 - elevation) / 90; };
  let content = '';
  [0, 30, 60].forEach(function (elevation) {
    content += '<circle cx="' + cx + '" cy="' + cy + '" r="' + radiusOf(elevation).toFixed(1)
             + '" style="fill:none;stroke:' + ((elevation === 0) ? 'var(--muted)' : 'var(--line)') + '"/>';
    content += svgText(cx + 4, cy - radiusOf(elevation) + 12, elevation + '°', 'font-size="10" style="fill:var(--muted)"');
  });
  for (let azimuth = 0; azimuth < 360; azimuth += 30) {
    const angle = azimuth * Math.PI / 180;
    content += svgLine(cx, cy, cx + radius * Math.sin(angle), cy - radius * Math.cos(angle), 'stroke:var(--line)');
  }
  [['С', 0], ['В', 90], ['Ю', 180], ['З', 270]].forEach(function (mark) {
    const angle = mark[1] * Math.PI / 180;
    content += svgText(cx + (radius + 16) * Math.sin(angle), cy - (radius + 16) * Math.cos(angle) + 4, mark[0],
                       'font-size="12" font-weight="700" style="fill:var(--muted)" text-anchor="middle"');
  });
  placed.forEach(function (sat) {
    const angle = sat.az * Math.PI / 180;
    const x = cx + radiusOf(sat.el) * Math.sin(angle);
    const y = cy - radiusOf(sat.el) * Math.cos(angle);
    const cls = prnClass(parseInt(String(sat.sat).slice(1), 10));
    content += '<circle cx="' + x.toFixed(1) + '" cy="' + y.toFixed(1) + '" r="6" class="fill-' + cls + '"/>';
    content += svgText(x + 9, y + 4, String(sat.sat), 'font-size="11" style="fill:var(--fg)"');
  });
  const inSolution = skyStatus.filter(function (sat) { return sat.pvt; }).length;
  const notes = ['НКА с вычисленным положением: ' + placed.length + ' из ' + skyStatus.length,
                 'НКА в решении: ' + inSolution];
  if (!placed.length) {
    notes.push('Положения НКА не вычислены: приёмнику нужны эфемериды');
    notes.push('и собственное решение. Цифровая информация модели нулевая;');
    notes.push('вид будет отлажен вместе с блоком генерации навигационного сообщения.');
  }
  notes.forEach(function (text, index) {
    content += svgText(500, 110 + index * 22, text, 'font-size="12.5" style="fill:'
                       + ((index < 2) ? 'var(--fg)' : 'var(--muted)') + '"');
  });
  box.innerHTML = svgFrame(width, height, 'Положения НКА на небосводе', sessionSubtitle(), content);
}

function renderSolutionView(box) {
  if (!receiverOpen()) {
    showHint(box, 'связь с приёмником не установлена');
    return;
  }
  const solution = solutionStatus;
  if (!solution) {
    showHint(box, 'решение ожидается от приёмника');
    return;
  }
  const none = 'нет данных';
  const stack = document.createElement('div');
  const tiles = document.createElement('div');
  stack.className = 'rxstack';
  tiles.className = 'tiles';
  tile(tiles, 'Статус решения', solution.fixed ? 'получено' : 'нет', solution.fixed ? '' : 'warn');
  // Без решения приёмник выводит своё время без привязки (1970-01-01), временем решения оно не является
  tile(tiles, 'Время решения', solution.fixed ? solution.time : none);
  tile(tiles, 'Широта, °', (solution.fixed && isFinite(solution.lat)) ? numberRu(solution.lat, 6) : none);
  tile(tiles, 'Долгота, °', (solution.fixed && isFinite(solution.lon)) ? numberRu(solution.lon, 6) : none);
  tile(tiles, 'Высота, м', (solution.fixed && isFinite(solution.height)) ? numberRu(solution.height, 1) : none);
  tile(tiles, 'НКА в решении из отслеживаемых', String(solution.used).replace('/', ' из '));
  stack.appendChild(tiles);
  if (!solution.fixed) {
    const hint = document.createElement('div');
    hint.className = 'hint';
    hint.textContent = 'Решения нет: приёмнику нужны эфемериды, а цифровая информация модели нулевая. '
                     + 'Вид будет отлажен вместе с блоком генерации навигационного сообщения.';
    stack.appendChild(hint);
  }
  box.textContent = '';
  box.appendChild(stack);
}
)PANEL";
} // namespace

const char*panelReceiverViewsScript() {
   return receiverViewsScript;
}
} // namespace glonass_service
