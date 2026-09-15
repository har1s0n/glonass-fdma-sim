#include "panel_receiver.h"

namespace glonass_service {
namespace {
constexpr const char* receiverScript =
   R"PANEL(
// ─── Компоновка «Приёмник» ───
// Живые данные внешнего приёмника PocketSDR: браузер подписывается на WebSocket приложения
// pocket_web. Сервис модели в обмене не участвует: приёмный тракт в его границы не входит.
// Потоковый сеанс открывается и закрывается точками того же источника.

// Период ДК L1OCd на выходе почипового уплотнения, мс: T_d = 2·N_d/f_T1 = 2 мс
// ([ИКД-L1OC] п. 2.2.1; А_L1OC.2)
const codePeriodMs = 2 * codeLengthD * 1000 / chipRateL1OC;
// Один чип уплотнения 1/f_T1, мс
const chipDurationMs = 1000 / chipRateL1OC;
// Допуски оценки канала: по кодовому смещению один чип уплотнения,
// по доплеровскому сдвигу 100 Гц
const coffToleranceChips = 1;
const dopToleranceHz = 100;
// Порт pocket_web по умолчанию: закрепляется за приёмником в описании состава стенда
// (решение от 14.09.2026: 8080 сервис сигнала L1OC, 8081 pocket_web, 8082 сервис модели АФУ)
const receiverPortDefault = 8081;
// Версия протокола pocket_web, под которую написан разбор
const receiverProtocol = 1;
// Обозначение сигнала L1OCd в PocketSDR (ключ -sig G1OCD) и код формата CS16 (SDR_FMT_CS16)
const receiverSignal = 'G1OCD';
const receiverFormatCs16 = 8;
// Отбор каналов: сопровождаемые не менее 2 с, как в таблице каналов интерфейса PocketSDR
const channelMinLockSeconds = 2;

const receiverViews = [
  { id: 'table', tab: 'Таблица каналов', alt: 'Таблица каналов приёмника',
    block: 'Внешний приёмник PocketSDR',
    what: 'Каналы, которые приёмник сопровождает не менее 2 с: C/N0, кодовое смещение COFF, доплеровский сдвиг DOP, флаги синхронизации, счётчики строк, ошибок и срывов. Отклонения ΔCOFF и ΔDOP отсчитываются от ожидаемых значений по параметрам открытого сеанса; НКА состава без сопровождения показаны отдельными строками.',
    why: 'канала распространения в модели нет и шума нет, поэтому приёмник удерживает и кросс-корреляционные пики: сопровождение само по себе не доказывает приём своего НКА.',
    look: 'канал истинный, если НКА входит в состав J, |ΔCOFF| меньше одного чипа уплотнения и |ΔDOP| меньше 100 Гц. Флаги SYNC: S оверлейный код, B символьная синхронизация, F строчная синхронизация, R инверсия полярности.' },
  { id: 'cn0', tab: 'C/N0 по НКА', alt: 'Отношение несущая/шум по номерам НКА',
    block: 'Внешний приёмник PocketSDR',
    what: 'Отношение несущая/шум по номерам НКА; цвет столбца показывает оценку канала, номера НКА состава выделены.',
    why: 'показывает действие состава J и амплитуды выбранного НКА на приём: ослабленный или исключённый источник теряет превышение над соседями.',
    look: 'шума в тракте нет, поэтому абсолютные значения условны, значимы сдвиги между сеансами. При нескольких источниках C/N0 ложного канала бывает выше, чем истинного: оценку даёт совпадение COFF и DOP, а не высота столбца.' },
  { id: 'match', tab: 'Сопоставление с моделью', alt: 'Отклонения показаний приёмника от ожидаемых',
    block: 'Внешний приёмник PocketSDR',
    what: 'Каждый сопровождаемый канал показан точкой: по горизонтали ΔCOFF в чипах уплотнения, по вертикали ΔDOP в герцах. Прямоугольник отмечает допуск ±1 чип и ±100 Гц; точки за пределами шкалы прижаты к её краю и не закрашены.',
    why: 'истинность канала определяется совпадением показаний приёмника с конфигурацией модели: ожидаемое кодовое смещение задаёт привязка n₀, ожидаемый доплеровский сдвиг задаёт расстройка Δf.',
    look: 'истинные каналы собираются в центре прямоугольника, ложные разбросаны по всему периоду ДК. COFF_ож = (−n₀/Fs) mod T_d, DOP_ож = Δf.' }
];
const classNames = { genuine: 'истинный', spurious: 'ложный', outside: 'вне состава',
                     missing: 'нет сопровождения', unknown: 'сеанс не открыт' };
const classOrder = ['genuine', 'spurious', 'outside', 'missing'];

let receiverSocket = null;
let receiverActive = false;
let receiverRetry = 1000;
let receiverTimer = 0;
let receiverHello = null;
let receiverConfig = null;
let receiverChannels = null;
let receiverStatus = null;
let receiverSession = null;
let receiverNotice = '';
let receiverView = 'table';

// Хранилище браузера может быть недоступно (частный режим): страница работает и без него
function storageGet(key) {
  try { return window.localStorage.getItem(key); } catch (error) { return null; }
}

function storageSet(key, value) {
  try {
    if (value === null) { window.localStorage.removeItem(key); } else { window.localStorage.setItem(key, value); }
  } catch (error) {
    // хранилище недоступно: значение живёт до перезагрузки страницы
  }
}

function receiverAddress() {
  const text = el('rxAddr').value.trim();
  return text || (location.hostname + ':' + receiverPortDefault);
}

function receiverOpen() {
  return !!receiverSocket && (receiverSocket.readyState === WebSocket.OPEN);
}

function receiverSend(command) {
  if (receiverOpen()) { receiverSocket.send(JSON.stringify(command)); }
}

// Таблица каналов раз в 200 мс, состояние приёмника раз в секунду, настройки однократно; эти темы
// нужны плиткам и держатся при любом виде. Темы остальных видов запрашиваются только на время
// показа вида, как в интерфейсе PocketSDR
function receiverSubscribe() {
  receiverSend({ cmd: 'sub', topic: 'ch_stat', cyc: 200, sys: 'ALL', chno: 0,
                 min_lock: channelMinLockSeconds, rfch: 0, opt: 0 });
  receiverSend({ cmd: 'sub', topic: 'rcv_stat', cyc: 1000 });
  receiverSend({ cmd: 'get', topic: 'cfg' });
  receiverViewSubscribe();
}

function receiverUnsubscribe() {
  receiverSend({ cmd: 'unsub', topic: 'ch_stat' });
  receiverSend({ cmd: 'unsub', topic: 'rcv_stat' });
  receiverViewUnsubscribe();
}

// Перерисовка не чаще раза в 100 мс: темы корреляторов и спектра приходят по нескольку раз в секунду
let receiverRenderPending = false;

function requestReceiverRender() {
  if (receiverRenderPending) { return; }
  receiverRenderPending = true;
  setTimeout(function () {
    receiverRenderPending = false;
    renderReceiver();
  }, 100);
}

function receiverLinkText(text) {
  el('rxLink').textContent = text;
}

function receiverConnect() {
  clearTimeout(receiverTimer);
  if (receiverSocket) { return; }
  let socket = null;
  try {
    socket = new WebSocket('ws://' + receiverAddress() + '/ws');
  } catch (error) {
    receiverLinkText('адрес не принят');
    return;
  }
  receiverSocket = socket;
  socket.binaryType = 'arraybuffer'; // двоичные кадры спектра и корреляторов (протокол v1, Binary frames)
  receiverLinkText('подключение');
  socket.onopen = function () {
    receiverRetry = 1000;
    receiverLinkText('связь установлена');
    if (receiverActive) { receiverSubscribe(); }
    renderReceiver();
  };
  socket.onmessage = function (event) {
    if (typeof event.data !== 'string') {
      if (receiverBinaryMessage(event.data)) { requestReceiverRender(); }
      return;
    }
    let message = null;
    try { message = JSON.parse(event.data); } catch (error) { return; }
    receiverMessage(message);
  };
  socket.onclose = function () {
    if (receiverSocket === socket) { receiverSocket = null; }
    receiverHello = null;
    receiverConfig = null;
    receiverChannels = null;
    receiverStatus = null;
    receiverViewsReset();
    receiverLinkText('связь не установлена');
    renderReceiver();
    if (receiverActive && !receiverSocket) {
      receiverTimer = setTimeout(receiverConnect, receiverRetry);
      receiverRetry = Math.min(receiverRetry * 2, 5000);
    }
  };
}

function receiverMessage(message) {
  if (!message || (typeof message.type !== 'string')) { return; }
  if (message.type === 'hello') {
    receiverHello = message;
    receiverSend({ cmd: 'get', topic: 'cfg' }); // приветствие рассылается и при пуске и останове приёмника
  } else if (message.type === 'cfg') {
    receiverConfig = message;
  } else if (message.type === 'ch_stat') {
    receiverChannels = parseChannelTable(String(message.str || ''));
    receiverViewChannelsChanged();
  } else if (message.type === 'rcv_stat') {
    receiverStatus = parseReceiverStatus(String(message.str || ''));
  } else if ((message.type === 'ack') && ((message.cmd === 'start') || (message.cmd === 'stop'))) {
    if (message.ok) { return; }
    receiverNotice = 'приёмник: команда ' + message.cmd + ' не выполнена' + (message.msg ? (': ' + message.msg) : '');
  } else if (!receiverExtraMessage(message)) {
    return;
  }
  requestReceiverRender();
}

// Таблица каналов (формат print_ch_stat, src/sdr_rcv.c): строка состояния, строка заголовка,
// по строке на канал. Пустая полоса C/N0 пропадает при разбиении по пробелам и восстанавливается
// пустым полем, как в интерфейсе PocketSDR.
function parseChannelTable(text) {
  const lines = text.split('\n');
  const head = /BUFF: *(\d+)% SRCH: *(\d+) LOCK: *(\d+)\/ *(\d+)/.exec(lines[0] || '');
  const rows = [];
  for (let i = 2; i < lines.length; ++i) {
    const field = lines[i].trim().split(/\s+/);
    if (field.length === 15) { field.splice(7, 0, ''); }
    if (field.length < 16) { continue; }
    rows.push({ ch: +field[0], sig: field[3], prn: +field[4], lock: +field[5], cn0: +field[6],
                coff: +field[8], dop: +field[9], sync: field[11], nav: +field[12], err: +field[13],
                lol: +field[14] });
  }
  return { buffer: head ? +head[1] : NaN, locked: head ? +head[3] : NaN,
           total: head ? +head[4] : NaN, rows: rows };
}

// Строка состояния приёмника (формат sdr_rcv_rcv_stat, src/sdr_rcv.c): время приёмника, с;
// источник; формат; гетеродины; квадратуры; Fs, МГц; каналы; поток, Мвыб/с; буфер, %
function parseReceiverStatus(text) {
  const field = text.trim().split(/\s+/);
  return { time: parseFloat(field[0]), rate: parseFloat(field[8]) };
}

function portOf(address) {
  const match = /:(\d+)$/.exec(String(address || ''));
  return match ? +match[1] : 0;
}

function membersOf(parameters) {
  const list = [];
  if (parameters.single) {
    list.push(parameters.pick);
    return list;
  }
  for (let j = 1; j <= parameters.count; ++j) { list.push(j); }
  return list;
}

// Приведение к периоду ДК L1OCd: остаток в [0, T_d), мс
function wrapCodePeriod(valueMs) {
  const rest = valueMs % codePeriodMs;
  return (rest < 0) ? (rest + codePeriodMs) : rest;
}

function expectedCoffMs(parameters) {
  return wrapCodePeriod(-parameters.startSample * 1000 / parameters.sampleRate);
}

// Круговая разность кодовых смещений, мс, в пределах [−T_d/2, T_d/2)
function coffDeviationMs(measuredMs, expectedMs) {
  return wrapCodePeriod(measuredMs - expectedMs + codePeriodMs / 2) - codePeriodMs / 2;
}

// Оценка канала: НКА вне состава J сеанса; истинный, если кодовое
// смещение и доплеровский сдвиг совпадают с ожидаемыми в пределах допусков; иначе ложный захват
function classifyChannel(row, session) {
  if (!session) { return 'unknown'; }
  if (session.members.indexOf(row.prn) < 0) { return 'outside'; }
  const coffMatches = Math.abs(coffDeviationMs(row.coff, session.expectedCoff) / chipDurationMs)
                      < coffToleranceChips;
  const dopMatches = Math.abs(row.dop - session.expectedDop) < dopToleranceHz;
  return (coffMatches && dopMatches) ? 'genuine' : 'spurious';
}

function sessionOf(stored) {
  return { id: stored.id, port: stored.port, query: stored.query, params: stored.params,
           members: membersOf(stored.params), expectedCoff: expectedCoffMs(stored.params),
           expectedDop: stored.params.residualHz };
}

function restoreSession() {
  const text = storageGet('rxSession');
  if (!text) { return; }
  try {
    const stored = JSON.parse(text);
    if (stored && stored.id && stored.params && (typeof stored.query === 'string')) {
      receiverSession = sessionOf(stored);
      receiverNotice = 'сеанс восстановлен из браузера: если приёмник его завершил, закройте сеанс';
    }
  } catch (error) {
    storageSet('rxSession', null);
  }
}

function errorMessageOf(text) {
  try { return JSON.parse(text).message; } catch (error) { return text; }
}

// Приёмник допускает команды пуска и останова, если приложение передало ему настройки
function receiverControllable() {
  return receiverOpen() && !!receiverHello && !!receiverHello.cfg_ena;
}

// Параметры сеанса фиксируются при открытии и служат опорой оценки каналов.
// Приёмник останавливается до открытия сеанса и запускается после: его отсчётное время начинается
// с первого отсчёта сеанса n₀. Переподключённый без перезапуска приёмник продолжает время прошлого
// сеанса, и опора COFF смещается на произвольную величину
async function openSession() {
  if (receiverSession) { return; }
  const parameters = currentParams();
  const restart = receiverControllable();
  if (receiverConfig) {
    const rate = Math.round(receiverConfig.fs * 1e6);
    if (rate !== parameters.sampleRate) {
      receiverNotice = 'сеанс не открыт: Fs модели ' + numberRu(parameters.sampleRate / 1e6, 3)
                     + ' МГц, приёмник настроен на ' + numberRu(rate / 1e6, 3) + ' МГц';
      renderReceiver();
      return;
    }
  }
  const query = queryOf(parameters);
  if (restart) { receiverSend({ cmd: 'stop' }); }
  try {
    const response = await fetch('/v1/stream/tcp?' + query + '&format=cs16', { method: 'POST' });
    const text = await response.text();
    if (!response.ok) {
      receiverNotice = 'сеанс не открыт, код ' + response.status + ': ' + errorMessageOf(text);
    } else {
      const reply = JSON.parse(text);
      const stored = { id: reply.sessionId, port: reply.port, query: query, params: parameters };
      receiverSession = sessionOf(stored);
      storageSet('rxSession', JSON.stringify(stored));
      receiverViewsReset(); // строки НС и кадры прошлого сеанса к новому не относятся
      receiverNotice = restart ? ''
        : 'приёмник не перезапущен: управление им недоступно; опора COFF верна, только если он запущен под этот сеанс';
    }
  } catch (error) {
    receiverNotice = 'обращение не выполнено: ' + String(error);
  }
  if (restart) { receiverSend({ cmd: 'start' }); }
  renderReceiver();
}

// Сеанс закрывается раньше останова приёмника: иначе приёмник, разорвав соединение, завершил бы
// сеанс сам. Остановленный приёмник не подхватит чужой сеанс на том же порту.
async function closeSession() {
  if (!receiverSession) { return; }
  try {
    const response = await fetch('/v1/stream/tcp/' + encodeURIComponent(receiverSession.id),
                                 { method: 'DELETE' });
    if (response.ok || (response.status === 404)) {
      receiverNotice = (response.status === 404) ? 'сеанс уже был завершён сервисом' : '';
      receiverSession = null;
      storageSet('rxSession', null);
      if (receiverControllable()) { receiverSend({ cmd: 'stop' }); }
    } else {
      receiverNotice = 'сеанс не закрыт, код ' + response.status;
    }
  } catch (error) {
    receiverNotice = 'обращение не выполнено: ' + String(error);
  }
  renderReceiver();
}

function receiverWarnings() {
  const list = [];
  if (receiverHello && (receiverHello.proto !== receiverProtocol)) {
    list.push('протокол приёмника версии ' + receiverHello.proto + ' не поддерживается: разбор написан под версию '
              + receiverProtocol);
  }
  if (receiverHello && !receiverHello.run) {
    list.push('приёмник остановлен: пуск выполняется в его собственном интерфейсе');
  }
  if (receiverConfig) {
    const rate = Math.round(receiverConfig.fs * 1e6);
    if (rate !== currentParams().sampleRate) {
      list.push('Fs модели не совпадает с частотой приёмника ' + numberRu(rate / 1e6, 3) + ' МГц');
    }
    if (receiverConfig.fmt !== receiverFormatCs16) {
      list.push('приёмник настроен не на формат CS16');
    }
    const port = portOf(receiverConfig.file);
    if (receiverSession && port && (port !== receiverSession.port)) {
      list.push('приёмник ожидает поток на порту ' + port + ', сеанс открыт на порту ' + receiverSession.port);
    }
  }
  if (receiverSession && (queryOf(currentParams()) !== receiverSession.query)) {
    list.push('органы управления отличаются от параметров сеанса: оценка каналов ведётся по сеансу; '
              + 'для применения закройте сеанс и откройте заново');
  }
  return list;
}

function renderSessionBox() {
  const session = receiverSession;
  const lines = receiverWarnings();
  el('rxSession').textContent = session
    ? ('сеанс ' + session.id + ', порт ' + session.port + ' · ' + summaryOf(session.params))
    : 'сеанс не открыт';
  el('rxOpen').disabled = !!session;
  el('rxClose').disabled = !session;
  if (receiverNotice) { lines.unshift(receiverNotice); }
  el('rxWarn').textContent = lines.join('\n');
}

// Сопровождаемые каналы L1OCd с оценкой и строки НКА состава без сопровождения
function channelRows() {
  const rows = [];
  if (receiverChannels) {
    receiverChannels.rows.forEach(function (row) {
      if (row.sig === receiverSignal) { rows.push({ row: row, cls: classifyChannel(row, receiverSession) }); }
    });
  }
  if (receiverSession) {
    receiverSession.members.forEach(function (prn) {
      for (let i = 0; i < rows.length; ++i) {
        if (rows[i].row.prn === prn) { return; }
      }
      rows.push({ row: { prn: prn }, cls: 'missing' });
    });
  }
  rows.sort(function (a, b) { return a.row.prn - b.row.prn; });
  return rows;
}

function countByClass(rows) {
  const counts = { genuine: 0, spurious: 0, outside: 0, missing: 0, unknown: 0 };
  rows.forEach(function (item) { counts[item.cls] += 1; });
  return counts;
}

function renderReceiverTiles(rows) {
  const box = el('rxTiles');
  const linked = receiverOpen();
  const running = linked && receiverHello && receiverHello.run;
  const counts = countByClass(rows);
  const none = 'нет данных';
  box.textContent = '';
  tile(box, 'Связь с приёмником', linked ? (running ? 'установлена' : 'приёмник остановлен') : 'нет',
       running ? '' : 'warn');
  tile(box, 'Время приёмника, с',
       (receiverStatus && isFinite(receiverStatus.time)) ? numberRu(receiverStatus.time, 1) : none);
  tile(box, 'Каналов в сопровождении', (receiverChannels && isFinite(receiverChannels.locked))
       ? (numberRu(receiverChannels.locked) + ' из ' + numberRu(receiverChannels.total)) : none);
  tile(box, 'Истинных в составе', receiverSession
       ? (numberRu(counts.genuine) + ' из ' + numberRu(receiverSession.members.length)) : none);
  tile(box, 'Ложных захватов', receiverSession ? numberRu(counts.spurious + counts.outside) : none);
  tile(box, 'Заполнение буфера, %',
       (receiverChannels && isFinite(receiverChannels.buffer)) ? numberRu(receiverChannels.buffer) : none);
  tile(box, 'Поток, Мвыб/с',
       (receiverStatus && isFinite(receiverStatus.rate)) ? numberRu(receiverStatus.rate, 1) : none);
}

function showHint(target, text) {
  const hint = document.createElement('div');
  hint.className = 'hint';
  hint.textContent = text;
  target.textContent = '';
  target.appendChild(hint);
}

function coffChipsOf(row) {
  return coffDeviationMs(row.coff, receiverSession.expectedCoff) / chipDurationMs;
}

function renderChannelTable(target, rows) {
  if (!rows.length) {
    showHint(target, 'приёмник не сопровождает ни одного канала');
    return;
  }
  const wrap = document.createElement('div');
  const table = document.createElement('table');
  const headRow = table.createTHead().insertRow();
  const body = table.createTBody();
  const titles = ['НКА', 'Состав J', 'Оценка', 'C/N0, дБ·Гц', 'COFF, мс', 'ΔCOFF, чип', 'DOP, Гц',
                  'ΔDOP, Гц', 'SYNC', 'Строк', 'Ошибок', 'Срывов', 'Сопровождение, с'];
  wrap.className = 'rxwrap';
  table.className = 'rxtable';
  titles.forEach(function (text, index) {
    const cell = document.createElement('th');
    cell.textContent = text;
    if (index <= 2) { cell.className = 'l'; }
    headRow.appendChild(cell);
  });
  rows.forEach(function (item) {
    const row = item.row;
    const line = body.insertRow();
    const tracked = (item.cls !== 'missing');
    const session = receiverSession;
    const cells = [
      String(row.prn),
      session ? ((session.members.indexOf(row.prn) >= 0) ? 'да' : 'нет') : '',
      classNames[item.cls],
      tracked ? numberRu(row.cn0, 1) : '',
      tracked ? numberRu(row.coff, 5) : '',
      (tracked && session) ? signedRu(coffChipsOf(row), 3) : '',
      tracked ? signedRu(row.dop, 1) : '',
      (tracked && session) ? signedRu(row.dop - session.expectedDop, 1) : '',
      tracked ? row.sync : '',
      tracked ? numberRu(row.nav) : '',
      tracked ? numberRu(row.err) : '',
      tracked ? numberRu(row.lol) : '',
      tracked ? numberRu(row.lock, 1) : ''
    ];
    cells.forEach(function (text, index) {
      const cell = line.insertCell();
      cell.textContent = text;
      if (index === 2) { cell.className = 'l cls-' + item.cls; } else if (index < 2) { cell.className = 'l'; }
    });
  });
  wrap.appendChild(table);
  target.textContent = '';
  target.appendChild(wrap);
}

function escapeText(text) {
  return String(text).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}

function svgText(x, y, text, attributes) {
  return '<text x="' + x.toFixed(1) + '" y="' + y.toFixed(1) + '" ' + (attributes || '') + '>'
       + escapeText(text) + '</text>';
}

function svgLegend(x, y, counts) {
  let out = '';
  let position = x;
  classOrder.forEach(function (cls) {
    if (!counts[cls]) { return; }
    const label = classNames[cls] + ': ' + counts[cls];
    out += '<rect x="' + position.toFixed(1) + '" y="' + (y - 9).toFixed(1)
         + '" width="10" height="10" rx="2" class="fill-' + cls + '"/>';
    out += svgText(position + 14, y, label, 'font-size="12" style="fill:var(--fg)"');
    position += 26 + label.length * 6.6;
  });
  return out;
}

function svgFrame(width, height, title, subtitle, content) {
  return '<svg class="rxsvg" viewBox="0 0 ' + width + ' ' + height + '" role="img">'
       + svgText(16, 24, title, 'font-size="15" font-weight="600" style="fill:var(--fg)"')
       + svgText(16, 43, subtitle, 'font-size="12" style="fill:var(--muted)"')
       + content + '</svg>';
}

function sessionSubtitle() {
  return receiverSession ? summaryOf(receiverSession.params) : 'сеанс не открыт: оценка каналов не выполняется';
}

function renderCn0Chart(target, rows) {
  const width = 960;
  const height = 380;
  const left = 58;
  const right = 18;
  const top = 64;
  const bottom = 56;
  const plotWidth = width - left - right;
  const plotHeight = height - top - bottom;
  let lastPrn = +el('jcount').max;
  let highest = 0;
  rows.forEach(function (item) {
    lastPrn = Math.max(lastPrn, item.row.prn);
    if (item.cls !== 'missing') { highest = Math.max(highest, item.row.cn0); }
  });
  const yMax = Math.max(60, Math.ceil((highest + 5) / 10) * 10);
  const step = plotWidth / lastPrn;
  const yOf = function (value) { return top + plotHeight * (1 - value / yMax); };
  const members = receiverSession ? receiverSession.members : [];
  let content = '';
  for (let level = 0; level <= yMax; level += 10) {
    content += '<line x1="' + left + '" x2="' + (left + plotWidth) + '" y1="' + yOf(level).toFixed(1)
             + '" y2="' + yOf(level).toFixed(1) + '" style="stroke:var(--line)"/>';
    content += svgText(left - 8, yOf(level) + 4, numberRu(level), 'font-size="11" style="fill:var(--muted)" text-anchor="end"');
  }
  rows.forEach(function (item) {
    const center = left + (item.row.prn - 0.5) * step;
    if (item.cls === 'missing') {
      content += svgText(center, yOf(0) - 6, 'нет', 'font-size="10" style="fill:var(--muted)" text-anchor="middle"');
      return;
    }
    const barTop = yOf(Math.max(0, Math.min(item.row.cn0, yMax)));
    content += '<rect x="' + (center - step * 0.3).toFixed(1) + '" y="' + barTop.toFixed(1) + '" width="'
             + (step * 0.6).toFixed(1) + '" height="' + (yOf(0) - barTop).toFixed(1) + '" class="fill-'
             + item.cls + '"/>';
    content += svgText(center, barTop - 4, numberRu(item.row.cn0, 1), 'font-size="9.5" style="fill:var(--fg)" text-anchor="middle"');
  });
  for (let prn = 1; prn <= lastPrn; ++prn) {
    const member = members.indexOf(prn) >= 0;
    content += svgText(left + (prn - 0.5) * step, yOf(0) + 16, String(prn), 'font-size="11" text-anchor="middle" '
                       + (member ? 'font-weight="700" style="fill:var(--fg)"' : 'style="fill:var(--muted)"'));
  }
  content += '<line x1="' + left + '" x2="' + (left + plotWidth) + '" y1="' + yOf(0).toFixed(1) + '" y2="'
           + yOf(0).toFixed(1) + '" style="stroke:var(--muted)"/>';
  content += svgText(left + plotWidth / 2, height - 12, 'номер НКА (жирным выделены НКА состава J сеанса)',
                     'font-size="12" style="fill:var(--muted)" text-anchor="middle"');
  content += svgText(14, top + plotHeight / 2, 'C/N0, дБ·Гц', 'font-size="12" style="fill:var(--muted)" text-anchor="middle" '
                     + 'transform="rotate(-90 14 ' + (top + plotHeight / 2).toFixed(1) + ')"');
  content += svgLegend(width - 430, 24, countByClass(rows));
  target.innerHTML = svgFrame(width, height, 'C/N0 по номерам НКА', sessionSubtitle(), content);
}

function renderMatchChart(target, rows) {
  if (!receiverSession) {
    showHint(target, 'откройте сеанс: ожидаемые значения берутся из его параметров');
    return;
  }
  const width = 960;
  const height = 420;
  const left = 70;
  const right = 24;
  const top = 64;
  const bottom = 56;
  const plotWidth = width - left - right;
  const plotHeight = height - top - bottom;
  // Шкала: десять допусков в каждую сторону
  const xLimit = 10 * coffToleranceChips;
  const yLimit = 10 * dopToleranceHz;
  const xOf = function (value) { return left + (value / xLimit + 1) / 2 * plotWidth; };
  const yOf = function (value) { return top + (1 - (value / yLimit + 1) / 2) * plotHeight; };
  const tracked = rows.filter(function (item) { return item.cls !== 'missing'; });
  // Точки, попавшие в одно место (истинные каналы в центре, прижатые к краю шкалы), получают
  // общую подпись со списком номеров НКА
  const labels = {};
  let content = '';
  let clipped = 0;
  [-xLimit, -xLimit / 2, 0, xLimit / 2, xLimit].forEach(function (value) {
    content += '<line x1="' + xOf(value).toFixed(1) + '" x2="' + xOf(value).toFixed(1) + '" y1="' + top + '" y2="'
             + (top + plotHeight) + '" style="stroke:' + (value === 0 ? 'var(--muted)' : 'var(--line)') + '"/>';
    content += svgText(xOf(value), top + plotHeight + 16, signedRu(value), 'font-size="11" style="fill:var(--muted)" text-anchor="middle"');
  });
  [-yLimit, -yLimit / 2, 0, yLimit / 2, yLimit].forEach(function (value) {
    content += '<line x1="' + left + '" x2="' + (left + plotWidth) + '" y1="' + yOf(value).toFixed(1) + '" y2="'
             + yOf(value).toFixed(1) + '" style="stroke:' + (value === 0 ? 'var(--muted)' : 'var(--line)') + '"/>';
    content += svgText(left - 8, yOf(value) + 4, signedRu(value), 'font-size="11" style="fill:var(--muted)" text-anchor="end"');
  });
  content += '<rect x="' + xOf(-coffToleranceChips).toFixed(1) + '" y="' + yOf(dopToleranceHz).toFixed(1)
           + '" width="' + (xOf(coffToleranceChips) - xOf(-coffToleranceChips)).toFixed(1) + '" height="'
           + (yOf(-dopToleranceHz) - yOf(dopToleranceHz)).toFixed(1)
           + '" fill-opacity="0.08" stroke-dasharray="4 3" style="fill:var(--accent);stroke:var(--accent)"/>';
  tracked.forEach(function (item) {
    const dx = coffChipsOf(item.row);
    const dy = item.row.dop - receiverSession.expectedDop;
    const outside = (Math.abs(dx) > xLimit) || (Math.abs(dy) > yLimit);
    const px = xOf(Math.max(-xLimit, Math.min(xLimit, dx)));
    const py = yOf(Math.max(-yLimit, Math.min(yLimit, dy)));
    if (outside) { clipped += 1; }
    content += '<circle cx="' + px.toFixed(1) + '" cy="' + py.toFixed(1) + '" r="5" class="fill-' + item.cls + '"'
             + (outside ? ' fill-opacity="0.15" stroke-width="1.5" stroke="currentColor"' : '') + '/>';
    const key = Math.round(px / 12) + ':' + Math.round(py / 12);
    if (!labels[key]) { labels[key] = { x: px, y: py, list: [] }; }
    labels[key].list.push(item.row.prn);
  });
  Object.keys(labels).forEach(function (key) {
    const label = labels[key];
    const alongRight = label.x > left + plotWidth * 0.8;
    content += svgText(alongRight ? (label.x - 8) : (label.x + 8), label.y - 7, label.list.join(', '),
                       'font-size="10.5" style="fill:var(--fg)"' + (alongRight ? ' text-anchor="end"' : ''));
  });
  content += svgText(left + plotWidth / 2, height - 12, 'ΔCOFF, чипов уплотнения', 'font-size="12" style="fill:var(--muted)" text-anchor="middle"');
  content += svgText(16, top + plotHeight / 2, 'ΔDOP, Гц', 'font-size="12" style="fill:var(--muted)" text-anchor="middle" '
                     + 'transform="rotate(-90 16 ' + (top + plotHeight / 2).toFixed(1) + ')"');
  content += svgLegend(width - 430, 24, countByClass(tracked));
  if (clipped) {
    content += svgText(width - right, top - 6, 'за пределами шкалы: ' + clipped, 'font-size="11" style="fill:var(--muted)" text-anchor="end"');
  }
  const subtitle = 'COFF_ож = ' + numberRu(receiverSession.expectedCoff, 5) + ' мс · DOP_ож = Δf = '
                 + signedRu(receiverSession.expectedDop) + ' Гц · ' + summaryOf(receiverSession.params);
  target.innerHTML = svgFrame(width, height, 'Отклонения показаний приёмника от ожидаемых', subtitle, content);
}

function renderReceiverView(rows) {
  const target = canvasOf(el('rxSlot'));
  const view = receiverViewOf(receiverView);
  if (view.render) { // виды на темах corr, log, psd, sat_stat, pvt_sol сами проверяют наличие данных
    view.render(target, rows);
    return;
  }
  if (!receiverChannels) {
    showHint(target, receiverSocket ? 'данные приёмника ожидаются' : 'связь с приёмником не установлена');
    return;
  }
  if (receiverView === 'table') {
    renderChannelTable(target, rows);
  } else if (receiverView === 'cn0') {
    renderCn0Chart(target, rows);
  } else {
    renderMatchChart(target, rows);
  }
}

function renderReceiver() {
  renderSessionBox();
  if (!receiverActive) { return; }
  const rows = channelRows();
  renderReceiverTiles(rows);
  renderReceiverView(rows);
}

function receiverViewOf(id) {
  for (let i = 0; i < receiverViews.length; ++i) {
    if (receiverViews[i].id === id) { return receiverViews[i]; }
  }
  return receiverViews[0];
}

function selectReceiverView(id) {
  const buttons = el('rxViews').children;
  if (receiverActive) { receiverViewUnsubscribe(); }
  receiverView = id;
  for (let i = 0; i < buttons.length; ++i) {
    buttons[i].classList.toggle('on', buttons[i].dataset.view === id);
  }
  if (receiverActive) { receiverViewSubscribe(); }
  fillTip(el('rxSlot'), receiverViewOf(id));
  renderReceiverBar();
  renderReceiver();
}

// Подписка держится, пока компоновка на экране; соединение при уходе из неё не рвётся
function receiverLayoutChanged(active) {
  receiverActive = active;
  if (active) {
    fillTip(el('rxSlot'), receiverViewOf(receiverView));
    renderReceiverBar();
    if (receiverOpen()) { receiverSubscribe(); } else { receiverConnect(); }
  } else {
    clearTimeout(receiverTimer);
    receiverUnsubscribe();
  }
  renderReceiver();
}

function receiverParamsChanged() {
  renderSessionBox();
}

function receiverInit() {
  const views = el('rxViews');
  receiverViews.forEach(function (view) {
    const button = document.createElement('button');
    button.type = 'button';
    button.textContent = view.tab;
    button.title = view.alt;
    button.dataset.view = view.id;
    button.className = (view.id === receiverView) ? 'on' : '';
    button.onclick = function () { selectReceiverView(view.id); };
    views.appendChild(button);
  });
  attachHelp(el('rxSlot'));
  el('rxAddr').value = storageGet('rxAddr') || (location.hostname + ':' + receiverPortDefault);
  el('rxAddr').addEventListener('change', function () {
    storageSet('rxAddr', el('rxAddr').value.trim());
    receiverRetry = 250;
    if (receiverSocket) {
      receiverSocket.close(); // переподключение по новому адресу выполнит обработчик закрытия
    } else if (receiverActive) {
      receiverConnect();
    }
  });
  el('rxOpen').onclick = openSession;
  el('rxClose').onclick = closeSession;
  restoreSession();
  renderSessionBox();
}
)PANEL";
} // namespace

const char*panelReceiverScript() {
   return receiverScript;
}
} // namespace glonass_service
