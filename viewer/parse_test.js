// Host test for the viewer's record-handling core. Extracts the PARSE-CORE
// section out of index.html (the exact code the browser runs — not a copy),
// then drives it with viewer/replay.jsonl through the same line-buffering
// logic the Web Serial read loop uses. Run: node viewer/parse_test.js
"use strict";
const fs = require("fs");
const path = require("path");

const html = fs.readFileSync(path.join(__dirname, "index.html"), "utf8");
const m = html.match(/PARSE-CORE-BEGIN ====([\s\S]*?)\/\* ==== PARSE-CORE-END/);
if (!m) { console.error("FAIL: parse-core markers not found"); process.exit(1); }
const core = new Function(m[1].replace(/^[\s\S]*?\*\//, "") +
  "; return { parseLine, CSV_COLS, stToCsvRow };")();
const { parseLine, CSV_COLS, stToCsvRow } = core;

let pass = 0, fail = 0;
function check(name, ok, detail) {
  if (ok) { pass++; console.log(`  PASS  ${name}`); }
  else { fail++; console.log(`  FAIL  ${name}${detail ? " — " + detail : ""}`); }
}

// Feed a byte stream through the viewer's read-loop framing: accumulate,
// split on '\n', classify each complete line. Returns kind counts + records.
function runStream(bytes, chunk) {
  const counts = { empty: 0, error: 0, hdr: 0, org: 0, st: 0, unknown: 0,
                   msg: 0, cal: 0, lcal: 0 };
  const sts = [], msgs = [], cals = [], lcals = [];
  let buf = "";
  for (let off = 0; off < bytes.length; off += chunk) {
    buf += bytes.slice(off, off + chunk);
    let i;
    while ((i = buf.indexOf("\n")) >= 0) {
      const p = parseLine(buf.slice(0, i));
      buf = buf.slice(i + 1);
      counts[p.kind]++;
      if (p.kind === "st") sts.push(p.rec);
      if (p.kind === "lcal") lcals.push(p.rec);
      if (p.kind === "msg") msgs.push(p.rec);
      if (p.kind === "cal") cals.push(p.rec);
    }
  }
  return { counts, sts, msgs, cals, lcals };
}

const raw = fs.readFileSync(path.join(__dirname, "replay.jsonl"), "utf8");

console.log("[1] whole-file replay through read-loop framing (64-byte chunks)");
const a = runStream(raw, 64);
check("hdr count = 1", a.counts.hdr === 1, `got ${a.counts.hdr}`);
check("org count = 1", a.counts.org === 1, `got ${a.counts.org}`);
check("st count = 4", a.counts.st === 4, `got ${a.counts.st}`);
check("error count = 5 (truncated, garbage, non-object, missing-seq, msg-no-txt)",
      a.counts.error === 5, `got ${a.counts.error}`);
check("msg count = 2 (the third is malformed -> error)", a.counts.msg === 2, `got ${a.counts.msg}`);
check("cal count = 1", a.counts.cal === 1, `got ${a.counts.cal}`);
check("msg text intact", a.msgs[0].txt === "imu ICM-45686 @0x68: OK");
check("cal record carries bg/eul/alq/dur",
      Array.isArray(a.cals[0].bg) && a.cals[0].bg.length === 3 &&
      Array.isArray(a.cals[0].eul) && a.cals[0].alq === 1 &&
      typeof a.cals[0].dur === "number");
check("unknown count = 1 (future record type skipped)",
      a.counts.unknown === 1, `got ${a.counts.unknown}`);
check("empty count = 1", a.counts.empty === 1, `got ${a.counts.empty}`);
check("st records cover fst 1,2,3,4",
      JSON.stringify([...new Set(a.sts.map(r => r.fst))].sort()) === "[1,2,3,4]");
check("seq gap visible (3 missing between 2 and 4)",
      a.sts.some((r, i) => i > 0 && r.seq === a.sts[i - 1].seq + 2));

console.log("[2] mid-stream join: attach at byte 40, 7-byte chunks");
const b = runStream(raw.slice(40), 7);
check("torn first line becomes exactly one extra error",
      b.counts.error === a.counts.error + 1,
      `got ${b.counts.error} vs base ${a.counts.error}`);
check("hdr lost (was torn), st/org/msg/cal unaffected",
      b.counts.hdr === 0 && b.counts.org === 1 && b.counts.st === 4 &&
      b.counts.msg === 2 && b.counts.cal === 1,
      JSON.stringify(b.counts));

console.log("[3] CSV flattening");
check("CSV_COLS has no duplicates", new Set(CSV_COLS).size === CSV_COLS.length);
check(`every st row width = CSV_COLS.length (${CSV_COLS.length})`,
      a.sts.every(r => stToCsvRow(r).length === CSV_COLS.length));
const col = name => CSV_COLS.indexOf(name);
const run = a.sts.find(r => r.fst === 3);
const align = a.sts.find(r => r.fst === 1);
const attOnly = a.sts.find(r => r.fst === 4);
const rowRun = stToCsvRow(run), rowAlign = stToCsvRow(align), rowAtt = stToCsvRow(attOnly);
check("run: lat lands in 'lat' column", rowRun[col("lat")] === "40.1164012",
      `got '${rowRun[col("lat")]}'`);
check("run: rejected baro kbr=0 preserved", rowRun[col("kbr")] === "0");
check("run: ibr value in 'ibr' column", rowRun[col("ibr")] === "6.2");
check("run: p expands to p_n/p_e/p_d", rowRun[col("p_n")] === "0.12" &&
      rowRun[col("p_e")] === "0.1" && rowRun[col("p_d")] === "-0.8");
check("run: raw mag mgr expands to mgr_x/y/z", rowRun[col("mgr_x")] === "24.9" &&
      rowRun[col("mgr_y")] === "-6.2" && rowRun[col("mgr_z")] === "47.5");
check("align: absent mgr -> empty cell", rowAlign[col("mgr_x")] === "");
check("align: null lat -> empty cell", rowAlign[col("lat")] === "");
check("align: null bb -> empty cell", rowAlign[col("bb")] === "");
check("align: null ngp -> empty cell", rowAlign[col("ngp")] === "");
check("att-only: null p -> empty p_n cell", rowAtt[col("p_n")] === "");
check("att-only: q_w populated", rowAtt[col("q_w")] === "0.9997");
check("att-only: field-gated nmg -> empty, kmg=0 kept",
      rowAtt[col("nmg")] === "" && rowAtt[col("kmg")] === "0");
check("att-only: vertical channel ral lands in 'ral' column",
      rowAtt[col("ral")] === "132.5", `got '${rowAtt[col("ral")]}'`);
check("att-only: rvs in 'rvs' column", rowAtt[col("rvs")] === "-21.8");
check("run: ral/sra present alongside p_d",
      rowRun[col("ral")] === "2.7" && rowRun[col("sra")] === "0.72");
check("align: absent ral field -> empty cell", rowAlign[col("ral")] === "");
check("lmx still lands in its column", rowRun[col("lmx")] === "1510");
check("last column is cdef_3", CSV_COLS[CSV_COLS.length - 1] === "cdef_3" &&
      rowRun[col("cdef_3")] === "1.25");
check("link fields pass through", run.rssi === -72 && run.snr === 9.5 &&
      run.lre === 1 && run.ltx === 120 && run.lcrc === 1);
check("control fields pass through", run.cmode === 2 &&
      Array.isArray(run.cdef) && run.cdef[1] === 1.25);
check("cdef expands to 4 csv columns", rowRun[col("cdef_0")] === "-1.25" &&
      rowRun[col("cdef_2")] === "-1.25");
check("records without link fields leave the columns empty",
      rowAlign[col("rssi")] === "" && rowAlign[col("cmode")] === "");
check("lcal records classified", a.counts.lcal === 2);
check("lcal table fields pass through",
      a.lcals[0].fin === 0 && a.lcals[0].src === 1 && a.lcals[0].n === 3 &&
      a.lcals[0].pus[2] === 1900 && a.lcals[0].deg[0] === -10);
check("lcal stage record keeps entry order",
      a.lcals[1].src === 2 && a.lcals[1].deg[0] === 10.5 &&
      a.lcals[1].pus[1] === 1120);

console.log(`\n${pass}/${pass + fail} checks passed${fail ? " — FAILURES ABOVE" : ""}`);
process.exit(fail ? 1 : 0);
