#include "panel_page.h"

#include <string>

#include "glonass/types.h"
#include "panel_receiver.h"
#include "panel_receiver_views.h"
#include "request_params_l1oc.h"

namespace glonass_service {
namespace {
constexpr const char* pageHead =
   R"PANEL(<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Панель модели сигнала L1OC</title>
<style>
:root{
  color-scheme:dark;
  --bg:#080d11;                       /* .twinSceneLeftPanel: фон rgba(8,13,17,.84) без прозрачности */
  --card:#1a1a1a;                     /* .twinTelemetrySectionContent, .twinSectionSummaryContainer */
  --soft:#222222;                     /* .twinMetricTile, .twinTelemetrySectionHeader */
  --fg:#f4f8f4;                       /* .twinMetricTile strong */
  --muted:#aaaaaa;                    /* .twinMetricTile span, .twinMetricLabel */
  --line:#333333;                     /* .twinTelemetrySection, .twinTelemetryHeader */
  --edge:rgba(255,255,255,.06);       /* .twinMetricTile, .twinAfuLegendPanel */
  --accent:#4caf50;                   /* .twinTelemetryTitle, .twinTelemetrySettingsCheckbox */
  --accent-hi:#8bdc65;                /* .twinMetricTile.accent strong */
  --accent-tile:linear-gradient(135deg,rgba(76,175,80,.22),rgba(34,34,34,.95)); /* .twinMetricTile.accent */
  --accent-edge:rgba(76,175,80,.32);  /* .twinMetricTile.accent */
  --pill:rgba(76,175,80,.12);         /* .twinSectionDetailsButton */
  --pill-edge:rgba(76,175,80,.35);    /* .twinSectionDetailsButton */
  --pill-fg:#cfead1;                  /* .twinSectionDetailsButton */
  --pill-on:rgba(76,175,80,.22);      /* .twinSectionDetailsButton:hover */
  --pill-on-edge:rgba(76,175,80,.72); /* .twinSectionDetailsButton:hover */
  --warn:#f6c86f;                     /* .twinNoticeTriggerWarning */
  --warn-edge:rgba(246,200,111,.38);  /* .twinNoticeTriggerWarning */
  --mark:#7edcff;                     /* .twinNoticeTriggerInfo */
  --note:#c7c7c7;                     /* .twinAfuLegendPanel span */
  --note-bg:rgba(34,34,34,.8);        /* .twinAfuLegendPanel */
  --popup:rgba(30,30,30,.98);         /* .twinTelemetrySettingsDialog */
  --popup-edge:#444444;               /* .twinTelemetrySettingsDialog */
  --popup-shadow:0 4px 12px rgba(0,0,0,.5); /* .twinTelemetrySettingsDialog */
  --popup-fg:#dce3e1;                 /* .twinNoticeItem p */
  --chip:rgba(255,255,255,.08);       /* .twinNoticeModule */
  --chip-fg:#b9c7ca;                  /* .twinNoticeModule */
  --disabled:#778185;                 /* .twinNoticeTrigger:disabled */
  --focus:#d7e8ee;                    /* .twinSectionDetailsButton:focus-visible */
  --r-card:6px;                       /* .twinTelemetrySection */
  --r-tile:7px;                       /* .twinMetricTile */
  --r-popup:8px;                      /* .twinTelemetrySettingsDialog, .twinAfuLegendPanel */
  /* Производное: кадр /v1/frames остаётся белым; на панели его белый фон переходит в --card
     (1 − 0,9 = 0,1, то есть #1a1a1a), поворот на 180° возвращает оттенки линий после инверсии */
  --frame-filter:invert(.9) hue-rotate(180deg);
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);
     font:14px/1.5 Arial,sans-serif} /* .twinSceneLeftPanel */
button:focus-visible,input:focus-visible{outline:2px solid var(--focus);outline-offset:2px}
header{border-bottom:1px solid var(--line);padding:14px 20px} /* .twinTelemetryHeader */
h1{font-size:16px;font-weight:700;line-height:24px;color:var(--accent);margin:0 0 3px} /* .twinTelemetryTitle */
.origin{margin:0 0 10px;color:var(--muted);font-size:12px}
h2{font-size:14px;font-weight:700;line-height:21px;color:var(--accent);margin:0 0 10px} /* .twinTelemetrySectionTitle */
.segmented{display:flex;flex-wrap:wrap;gap:6px}
/* Переключателей в образце нет: кнопки-пилюли .twinSectionDetailsButton, выбранная в состоянии
   раскрытого блока, при наведении меняются рамка и текст (производное) */
.segmented button{font:inherit;font-size:11px;font-weight:800;line-height:1;min-height:28px;
  padding:6px 9px;border:1px solid var(--pill-edge);border-radius:999px;
  background:var(--pill);color:var(--pill-fg);cursor:pointer}
.segmented button:hover{border-color:var(--pill-on-edge);color:#fff}
.segmented button.on{background:var(--pill-on);border-color:var(--pill-on-edge);color:#fff}

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
body.layout-receiver main{grid-template-columns:300px minmax(0,1fr);
  grid-template-areas:"controls receiver"}
#receiver{grid-area:receiver}
body.layout-overview #tabs,body.layout-overview #single,body.layout-overview #compare{display:none}
body.layout-detail #grid,body.layout-detail #compare{display:none}
body.layout-compare #grid,body.layout-compare #single,body.layout-compare #metrics{display:none}
body.layout-receiver #metrics,body.layout-receiver #tabs,body.layout-receiver #grid,
body.layout-receiver #single,body.layout-receiver #compare{display:none}
body:not(.layout-receiver) #receiver,body:not(.layout-receiver) #session{display:none}

/* Левая колонка оформлена блоком .twinTelemetrySection: строка заголовка
   .twinTelemetrySectionHeader, содержимое .twinTelemetrySectionContent */
#controls{background:var(--card);border:1px solid var(--line);border-radius:var(--r-card);
  overflow:hidden}
#controls h2{margin:0;padding:10px 15px;background:var(--soft)}
.fields{display:grid;gap:13px;padding:15px}
body.layout-detail .fields,body.layout-compare .fields{
  grid-template-columns:repeat(auto-fit,minmax(240px,1fr));align-items:start}
.field label{display:flex;justify-content:space-between;gap:8px;font-size:12px;
  color:var(--muted);margin-bottom:4px} /* .twinMetricLabel */
.field .val{color:var(--fg);font-weight:700;white-space:nowrap}
/* Ползунков в образце нет: цвет по флажкам .twinTelemetrySettingsCheckbox, дорожка тёмная
   по color-scheme (производное) */
.field input[type=range]{width:100%;accent-color:var(--accent);margin:0}
.check{display:flex;align-items:center;gap:6px;font-size:12px;color:var(--muted);
  margin-top:6px;cursor:pointer}
.check input{accent-color:var(--accent);margin:0}
.act{font:inherit;font-size:11px;font-weight:800;line-height:1;min-height:28px;padding:6px 9px;
  border:1px solid var(--pill-edge);border-radius:999px;
  background:var(--pill);color:var(--pill-fg);cursor:pointer} /* .twinSectionDetailsButton */
.act:hover{background:var(--pill-on);border-color:var(--pill-on-edge);color:#fff}
.fields .act{justify-self:start} /* пилюля по ширине надписи, как у .twinSectionDetailsButton */

.tiles{display:grid;gap:8px;grid-template-columns:repeat(auto-fit,minmax(155px,1fr));
  align-content:start}
/* Плитка .twinMetricTile, текст по центру (в образце наследуется от #root); подпись
   переносится по строкам и занимает не менее двух строк, как у .twinAfuMetricsGrid */
.tile{background:var(--soft);border:1px solid var(--edge);border-radius:var(--r-tile);padding:8px;
  text-align:center}
.tile .k{font-size:11px;line-height:1.25;color:var(--muted);margin-bottom:4px;min-height:28px}
.tile .v{font-size:13px;font-weight:700;line-height:1.2}
.tile.warn{border-color:var(--warn-edge)}
.tile.warn .v{color:var(--warn)}
.tile.wide{grid-column:1/-1;border-color:var(--warn-edge)}
.tile.wide .v{font-size:13px;font-weight:400;color:var(--warn)}

#grid{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(450px,1fr))}
/* Карточка кадра оформлена блоком .twinTelemetrySection */
.slot{position:relative;background:var(--card);border:1px solid var(--line);
  border-radius:var(--r-card);padding:6px;min-height:140px;display:flex;align-items:center;
  justify-content:center}
.slot img{width:100%;height:auto;display:block;border-radius:4px;filter:var(--frame-filter)}
.canvas{width:100%;display:flex;align-items:center;justify-content:center}

/* Значок подсказки по кнопке .twinTelemetrySettingsButton, раскрытый по пилюле */
.help{position:absolute;top:9px;right:9px;width:20px;height:20px;padding:0;z-index:4;
  border:1px solid var(--line);border-radius:50%;background:var(--soft);color:var(--muted);
  font:700 12px/1 Arial,sans-serif;cursor:help}
.help:hover,.slot.tipOn .help{background:var(--pill-on);border-color:var(--pill-on-edge);color:#fff}
/* Карточка подсказки по диалогу .twinTelemetrySettingsDialog */
.tip{position:absolute;top:35px;right:9px;width:440px;max-width:calc(100% - 18px);z-index:5;
  text-align:left;cursor:default;
  background:var(--popup);border:1px solid var(--popup-edge);border-radius:var(--r-popup);
  padding:10px 12px;box-shadow:var(--popup-shadow);
  opacity:0;visibility:hidden;transform:translateY(-4px);
  transition:opacity .12s ease,transform .12s ease}
.help:hover + .tip,.help:focus-visible + .tip,.tip:hover,.slot.tipOn .tip{
  opacity:1;visibility:visible;transform:none}
.tip::before{content:"";position:absolute;top:-5px;right:13px;width:8px;height:8px;
  background:var(--popup);border-left:1px solid var(--popup-edge);
  border-top:1px solid var(--popup-edge);transform:rotate(45deg)}
/* Метка блока тракта по метке модуля .twinNoticeModule, без прописных: обозначения Ч3 */
.tipHead{display:inline-block;margin-bottom:7px;padding:2px 6px;border-radius:4px;
  background:var(--chip);color:var(--chip-fg);font-size:11px;font-weight:700;letter-spacing:.04em}
.tipText{margin:0 0 6px;font-size:12.5px;line-height:1.45;color:var(--popup-fg)}
.tipText:last-child{margin-bottom:0}
.tipWhy{color:var(--muted)}
.tipWhy b{color:var(--fg);font-weight:700}
.slot.busy{opacity:.5}
#grid .slot{cursor:pointer}
#single{display:grid;justify-items:center}
#single .slot{width:100%;max-width:972px}
.reject{color:var(--warn);font-size:13px;text-align:center;padding:24px 14px}
.hint{color:var(--muted);font-size:13px;text-align:center;padding:24px 14px}

#compare{display:grid;gap:12px;grid-template-columns:minmax(0,1fr) 230px minmax(0,1fr)}
#compare .col{display:grid;gap:8px;align-content:start}
.colhead{display:flex;align-items:center;justify-content:space-between;gap:8px;
  font-size:14px;font-weight:700;color:var(--accent)} /* .twinTelemetrySectionTitle */
.params{font-size:12px;color:var(--note);background:var(--note-bg);border:1px solid var(--edge);
  border-radius:var(--r-popup);padding:10px} /* .twinAfuLegendPanel */
#compare .tiles{grid-template-columns:repeat(auto-fit,minmax(140px,1fr))}
.col.diffs .tiles{grid-template-columns:minmax(0,1fr)}
/* Разность выводится выделенной плиткой .twinMetricTile.accent */
.col.diffs .tile{background:var(--accent-tile);border-color:var(--accent-edge)}
.col.diffs .tile .v{color:var(--accent-hi)}

/* Сеанс продолжает левую колонку вторым блоком со своей строкой заголовка */
#session{display:grid;gap:9px;padding:0 0 15px;border-top:1px solid var(--line)}
#session h2{margin:0}
#session>:not(h2){margin:0 15px}
/* Полей ввода в панели «Телеметрия» нет: фон и радиус плитки, рамка блока (производное) */
.field input[type=text]{width:100%;font:inherit;font-size:13px;padding:5px 8px;
  border:1px solid var(--line);border-radius:var(--r-tile);background:var(--soft);color:var(--fg)}
.rxrow{display:flex;flex-wrap:wrap;gap:6px}
.act:disabled{opacity:.72;cursor:default;color:var(--disabled);border-color:var(--line);
  background:var(--soft)} /* .twinNoticeTrigger:disabled */
.rxwarn{font-size:12px;line-height:1.45;color:var(--warn);white-space:pre-line}
.rxwarn:empty{display:none}
#receiver{display:grid;gap:12px;align-content:start}
#rxTiles{grid-template-columns:repeat(auto-fit,minmax(135px,1fr))}
#rxSlot{min-height:220px;padding:10px}
.rxwrap{width:100%;overflow-x:auto}
/* Таблиц в образце нет: заголовок по .twinMetricLabel, линии по рамкам блока и плитки (производное) */
.rxtable{width:100%;border-collapse:collapse;font-size:12px;font-variant-numeric:tabular-nums}
.rxtable th{font-weight:700;color:var(--muted);text-align:right;padding:4px 7px;
  border-bottom:1px solid var(--line);white-space:nowrap}
.rxtable td{text-align:right;padding:3px 7px;border-bottom:1px solid var(--edge);white-space:nowrap}
.rxtable .l{text-align:left}
.rxtable th:last-child{padding-right:34px}
.rxsvg{width:100%;height:auto;display:block}
.rxnote{margin:0;font-size:12px;color:var(--muted)}
/* Строка выбора над видом (НКА для корреляторов): поле как у полей ввода панели (производное) */
.rxbar{display:flex;flex-wrap:wrap;align-items:center;gap:8px;font-size:12px;color:var(--muted)}
.rxbar:empty{display:none}
.rxbar select{font:inherit;font-size:12px;padding:4px 8px;border:1px solid var(--line);
  border-radius:var(--r-tile);background:var(--soft);color:var(--fg)}
.rxstack{width:100%;display:grid;gap:10px}
.rxmono{font-family:monospace;font-size:11px}
.cls-genuine{color:var(--accent-hi);font-weight:700}
.cls-spurious{color:var(--warn);font-weight:700}
.cls-outside{color:var(--mark)}
.cls-missing,.cls-unknown{color:var(--muted)}
.fill-genuine{fill:var(--accent)}
.fill-spurious{fill:var(--warn)}
.fill-outside{fill:var(--mark)}
.fill-missing,.fill-unknown{fill:var(--muted)}

@media (max-width:1100px){
  body.layout-overview main{grid-template-columns:minmax(0,1fr);
    grid-template-areas:"controls" "metrics" "grid"}
  body.layout-receiver main{grid-template-columns:minmax(0,1fr);
    grid-template-areas:"controls" "receiver"}
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
<button type="button" data-layout="receiver">Приёмник</button>
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
<div id="session">
<h2>Сеанс и приёмник</h2>
<div class="field">
<label for="rxAddr">Адрес приёмника<span class="val" id="rxLink">связь не установлена</span></label>
<input type="text" id="rxAddr" spellcheck="false" autocomplete="off">
</div>
<div class="rxrow">
<button type="button" class="act" id="rxOpen">Открыть сеанс</button>
<button type="button" class="act" id="rxClose">Закрыть сеанс</button>
</div>
<div class="params" id="rxSession">сеанс не открыт</div>
<div class="rxwarn" id="rxWarn"></div>
</div>
</aside>

<div class="tiles" id="metrics"></div>
<div class="segmented" id="tabs"></div>
<div id="grid"></div>
<div id="single"><div class="slot" id="slotOne"><div class="canvas"></div></div></div>

<div id="compare">
<section class="col">
<div class="colhead"><span>Опорная конфигурация</span>
<button type="button" class="act" id="pin">Взять текущую</button></div>
<div class="slot" id="slotRef"><div class="canvas">
<div class="hint">опорная конфигурация не задана</div></div></div>
<div class="params" id="refParams">не задана</div>
<div class="tiles" id="refMetrics"></div>
</section>
<section class="col diffs">
<div class="colhead"><span>Разность</span></div>
<div class="tiles" id="diffMetrics"></div>
</section>
<section class="col">
<div class="colhead"><span>Текущая конфигурация</span></div>
<div class="slot" id="slotCur"><div class="canvas"></div></div>
<div class="params" id="curParams">строится</div>
<div class="tiles" id="curMetrics"></div>
</section>
</div>

<section id="receiver">
<div class="tiles" id="rxTiles"></div>
<div class="segmented" id="rxViews"></div>
<div class="rxbar" id="rxBar"></div>
<div class="slot" id="rxSlot"><div class="canvas">
<div class="hint">связь с приёмником не установлена</div></div></div>
<p class="rxnote">Шума в тракте нет: абсолютные значения C/N0 условны, значимы сдвиги между сеансами.</p>
</section>
</main>

<script>
'use strict';
)PANEL";

// Часть страницы после блока числовых констант сценария
constexpr const char* pageTail =
   R"PANEL(
const kinds = [
  { id: 'psd', tab: 'СПМ', alt: 'Спектральная плотность мощности',
    block: 'Блоки В, Г_L1OC, Д_L1OC',
    what: 'Оценка спектральной плотности мощности по отсчётам модели. Ось частот отсчитывается от опорной частоты f₀; штриховые линии показывают полосу модели B_model = 2·f_T1 = 2,046 МГц, то есть границу главного лепестка по первым нулям.',
    why: 'по занятой полосе и положению спектра видно действие несущей, модуляции и суммирования состава; смещение спектра отвечает расстройке Δf.',
    look: 'между штриховыми линиями лежит главный лепесток, за ними боковые со спадом 20 дБ на декаду. Условие представимости В.2 требует |Δf| + B_model ≤ Fs/2, отсюда нижняя граница Fs = 4,092 МГц.' },
  { id: 'waveform', tab: 'Осциллограмма', alt: 'Осциллограмма квадратур',
    block: 'Блоки А_L1OC, В, Г_L1OC',
    what: 'Квадратуры I и Q в окне 16 чипов уплотнения; чип уплотнения длится 977,5 нс. Заливкой отмечены чипы компоненты L1OCp.',
    why: 'показывает почиповое временное уплотнение вблизи: знаки чипов дальномерного кода и перетекание мощности из I в Q при повороте начальной фазы φ₀.',
    look: 'на чиповом интервале передаётся ровно одна компонента, компоненты не суммируются. При Δf = 0 и φ₀ = 0 квадратура Q нулевая; поворот φ₀ переливает в неё мощность, не меняя знаков чипов.' },
  { id: 'acf', tab: 'АКФ', alt: 'Периодическая автокорреляционная функция дальномерного кода',
    block: 'Блок А_L1OC',
    what: 'Периодическая автокорреляция дальномерного кода: совпадение кода с самим собой, сдвинутым на τ чипов. При τ = 0 совпадают все N чипов, это главный лепесток; при τ ≠ 0 остаются боковые.',
    why: 'кадр воспроизводит работу коррелятора приёмника, который перебирает задержку и берёт ту, где корреляция максимальна; превышение главного лепестка над боковыми задаёт запас различения истинной задержки от ложной.',
    look: 'максимум бокового лепестка: ДК_L1OCd (N = 1023) −23,939 дБ, ДК_L1OCp (N = 4092) −24,780 дБ. Кадр считается по таблицам кодов, поэтому Fs, Δf, n₀, φ₀ и амплитуды его не меняют. Код берётся у первого НКА состава: чтобы сменить его, включите «только выбранный НКА».' },
  { id: 'ccf', tab: 'ВКФ', alt: 'Огибающая периодической взаимнокорреляционной функции ансамбля',
    block: 'Блок А_L1OC',
    what: 'Огибающая взаимной корреляции: максимум |R(τ)| по всем парам состава. Показывает, насколько чужой код похож на искомый при любом сдвиге.',
    why: 'при кодовом разделении все НКА занимают одну полосу и разделяются только корреляцией; неподавленный остаток и есть помеха множественного доступа.',
    look: 'максимум ВКФ: ДК_L1OCd −23,939 дБ, ДК_L1OCp −24,074 дБ; норма [ИКД-общ] п. 5.1.5 ограничивает средний квадрат, и ансамбль лежит на границе Велча. Кадр зависит только от состава J (пар |J|·(|J|−1)/2); при |J| = 1 пар нет и кадр отклоняется.' },
  { id: 'navline', tab: 'Строка НС', alt: 'Кадр строки навигационного сообщения',
    block: 'Блок Б_L1OC',
    what: 'Символы строки на выходе свёрточного кода (133,171), зоны поля СМВ и проверочных бит ЦК, отметка текущего символа w[n₀].',
    why: 'единственный кадр навигационного сообщения: показывает, в какую строку и в какой её символ попадает привязка по времени n₀.',
    look: 'символ свёрточного кода длится 4 мс, нормальная строка содержит L_с = 500 символов, то есть 2 с модельного времени. Ползунок времени двигает отметку по строке, и каждые 2 с сменяется номер строки ℓ.' },
  { id: 'level', tab: 'Гистограмма', alt: 'Гистограмма мгновенных значений',
    block: 'Блок Д_L1OC',
    what: 'Распределение мгновенных значений квадратуры I после нормировки: 262 144 отсчёта, 128 корзин. Штриховая линия показывает границу шкалы ±η·ΣA_j, опорные линии отмечают ±среднеквадратичное значение.',
    why: 'нормировка η = 1/√(ΣA_j²) приводит среднюю мощность суммы к единице при любом составе: без неё выход менял бы масштаб с каждой сменой состава, а масштаб квантования CS16 пришлось бы подбирать заново.',
    look: 'при полном составе сумма 24 знакопеременных вкладов даёт колокол, пик отстоит от среднеквадратичного значения на 14,8 дБ. При одиночном НКА остаются два уровня ±1 и пик-фактор 0 дБ.' }
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

// Показатели берёт только точка режима А: они выводятся аналитически и стоят единицы
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

// Поле изображения внутри слота: значок подсказки и карточка лежат рядом и не стираются
// при перерисовке кадра
function canvasOf(slot) {
  return slot.querySelector('.canvas') || slot;
}

// Значок «?» в углу кадра. Наведение и фокус раскрывают карточку средствами стиля, нажатие
// удерживает её раскрытой, иначе на сенсорном экране подсказка недоступна
function attachHelp(slot) {
  const button = document.createElement('button');
  const tip = document.createElement('div');
  button.type = 'button';
  button.className = 'help';
  button.textContent = '?';
  button.setAttribute('aria-label', 'пояснение к кадру');
  button.setAttribute('aria-expanded', 'false');
  button.onclick = function (event) {
    event.stopPropagation(); // иначе сработал бы переход в разбор по клику на кадре
    button.setAttribute('aria-expanded', slot.classList.toggle('tipOn') ? 'true' : 'false');
  };
  tip.className = 'tip';
  tip.setAttribute('role', 'tooltip');
  tip.onclick = function (event) { event.stopPropagation(); };
  slot.appendChild(button);
  slot.appendChild(tip);
}

function fillTip(slot, kind) {
  const tip = slot.querySelector('.tip');
  if (!tip) { return; }
  const head = document.createElement('div');
  const what = document.createElement('p');
  head.className = 'tipHead';
  head.textContent = kind.block;
  what.className = 'tipText';
  what.textContent = kind.what;
  tip.textContent = '';
  tip.appendChild(head);
  tip.appendChild(what);
  tip.appendChild(labelledText('Зачем: ', kind.why));
  tip.appendChild(labelledText('Смотреть: ', kind.look));
}

function labelledText(label, value) {
  const paragraph = document.createElement('p');
  const marker = document.createElement('b');
  paragraph.className = 'tipText tipWhy';
  marker.textContent = label;
  paragraph.appendChild(marker);
  paragraph.appendChild(document.createTextNode(value));
  return paragraph;
}

function closeTips() {
  const opened = document.querySelectorAll('.slot.tipOn');
  for (let i = 0; i < opened.length; ++i) {
    const button = opened[i].querySelector('.help');
    opened[i].classList.remove('tipOn');
    if (button) { button.setAttribute('aria-expanded', 'false'); }
  }
}

function showReject(slot, status, message) {
  const box = document.createElement('div');
  const target = canvasOf(slot);
  box.className = 'reject';
  box.textContent = (status > 0) ? ('кадр отклонён, код ' + status + ': ' + message)
                                 : ('обращение не выполнено: ' + message);
  target.textContent = '';
  target.appendChild(box);
}

// Кадр запрашивается изображением и вставляется через <img>: каждый SVG остаётся отдельным
// документом, поэтому одинаковые внутренние идентификаторы шести кадров не сталкиваются.
async function loadFrame(slot, kind, query, token) {
  fillTip(slot, kind);
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
    const target = canvasOf(slot);
    image.alt = kind.alt;
    image.onload = function () { URL.revokeObjectURL(objectUrl); };
    image.src = objectUrl;
    target.textContent = '';
    target.appendChild(image);
  } catch (error) {
    if (token === frameToken) { showReject(slot, 0, String(error)); }
  } finally {
    slot.classList.remove('busy');
  }
}

function refreshFrames() {
  const query = queryOf(currentParams());
  const token = ++frameToken;
  if (layout === 'receiver') { return; } // кадры модели в компоновке «Приёмник» не строятся
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
  receiverLayoutChanged(name === 'receiver');
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
  receiverParamsChanged();
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
    const canvas = document.createElement('div');
    slot.className = 'slot';
    slot.id = 'slot-' + kind.id;
    slot.onclick = function () { selectKind(kind.id); setLayout('detail'); };
    canvas.className = 'canvas';
    slot.appendChild(canvas);
    attachHelp(slot);
    grid.appendChild(slot);
  });

  // Подсказка ставится у кадра текущей конфигурации; у опорного кадра «Сравнения» она была бы
  // повтором того же текста
  attachHelp(el('slotOne'));
  attachHelp(el('slotCur'));
  document.addEventListener('click', closeTips);
  document.addEventListener('keydown', function (event) {
    if (event.key === 'Escape') { closeTips(); }
  });

  el('fs').max = sampleRateSteps.length - 1;
  el('fs').value = Math.max(0, sampleRateSteps.indexOf(defaultSampleRate));

  ['jcount', 'jsingle', 'jpick', 'amp', 'fs', 'df', 'tm', 'phi'].forEach(function (id) {
    const control = el(id);
    control.addEventListener('input', function () { updateLabels(); scheduleState(); });
    control.addEventListener('change', function () { updateLabels(); refreshFrames(); });
    control.addEventListener('input', receiverParamsChanged);
  });

  const layoutButtons = el('layouts').children;
  for (let i = 0; i < layoutButtons.length; ++i) {
    layoutButtons[i].onclick = function () { setLayout(this.dataset.layout); };
  }
  el('pin').onclick = applyReference;
  el('reset').onclick = resetControls;
  receiverInit();

  updateLabels();
  renderDiff();
  loadInfo();
  refreshState();
  refreshFrames();
}
)PANEL";

// Завершение страницы. Вызов init() стоит после сценария компоновки «Приёмник»: объявления
// const этого сценария должны быть вычислены до первого обращения к ним
constexpr const char* pageEnd = R"PANEL(
init();
</script>
</body>
</html>
)PANEL";

std::string composePage() {
   std::string page = pageHead;

   page += "const carrierFreqHz = " + std::to_string(glonass::carrierFreqL1OC) + ";\n";
   page += "const sampleRateSteps = [" + std::to_string(2 * glonass::modelBandwidthL1OC)
           + ", 5000000, 10000000, " + std::to_string(glonass_params::defaultSampleRate)
           + ", 40000000];\n";
   page += "const defaultSampleRate = " + std::to_string(glonass_params::defaultSampleRate) + ";\n";

   // Компоновка «Приёмник»: N_d и f_T1 задают период ДК L1OCd на выходе уплотнения
   // T_d = 2·N_d/f_T1 и допуск по кодовому смещению в один чип уплотнения 1/f_T1
   page += "const codeLengthD = " + std::to_string(glonass::codeLengthD) + ";\n";
   page += "const chipRateL1OC = " + std::to_string(glonass::chipRateL1OC) + ";\n";
   page += pageTail;
   page += panelReceiverScript();
   page += panelReceiverViewsScript();
   page += pageEnd;
   return page;
}
} // namespace

const std::string &panelPageHtml() {
   static const std::string page = composePage();

   return page;
}
} // namespace glonass_service
