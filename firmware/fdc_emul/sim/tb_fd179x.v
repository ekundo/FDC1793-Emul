// Сверка ядра ВГ93 с паспортом FD179X-01 (Western Digital, октябрь 1979;
// https://www.bitsavers.org/components/westernDigital/FD179X-01_Data_Sheet_Oct1979.pdf).
// Раздел за разделом паспорта -- то, чего
// не проверяет tb_fdc1793.v (сброс, Restore, Seek, Read/Write Sector, Write
// Track, Read Address, Read Track, Force Interrupt немедленный).
//
// ЧТО ПРОВЕРЯЕТСЯ: rtl/fdc_core.v (ядро + кодер записи rtl/fdc/mfm_wr.v) на
// такте 16 МГц без деления. Интерфейс --
// как у самой микросхемы: A1-A0, CS, RE, WE, DRQ, INTRQ, HLD/HLT, STEP/DIRC,
// WG/WD, TR00, IP, WPRT, RAW READ. Выбор привода, стороны, мотор и готовность
// -- снаружи ядра, у каждой машины свои; здесь их нет.
//
// ТАКТ. Ядро на 16 МГц выдаёт времена ВГ93 на CLK = 1 МГц (пятидюймовый MFM,
// 250 кбит/с): по паспорту на 1 МГц все времена вдвое против таблиц для 2 МГц --
// шаги 6/12/20/30 мс, успокоение головки 30 мс, импульс шага от 4 мкс, установка
// направления 24 мкс.
//
// ЧТО ЕСТЬ «ПРОВАЛ», А ЧТО «ОТЛИЧИЕ». CHECK -- требование паспорта, которое ядро
// обязано выполнять. NOTE -- требование, о котором известно, что ядро его не
// выполняет (решено не чинить или нечем); такие строки печатаются как
// «пока нет», прогон из-за них не падает. Список -- в README.md в корне.
//
// МОДЕЛЬ НОСИТЕЛЯ. Дорожка: индекс 200 мкс, промежуток, NSEC секторов по 256
// байт (длина 01), между ними промежутки по формату IBM System 34. Номер
// дорожки в поле адреса -- позиция головки плюс id_trk_off (для проверки V),
// сторона -- id_side. Порча: bad_id_crc / bad_dt_crc (номер сектора, 0 --
// нет), deleted_sec -- метка F8 вместо FB, no_data_sec -- поле данных не
// записано. Данные сектора -- функция (дорожка, сектор, номер байта).
`timescale 1ns/1ps
`default_nettype none

module tb_fd179x;

reg clk = 0;
always #31.25 clk = ~clk;                        // 16 МГц

// ---------------------------------------------------------------- шина
reg        nres = 1'b0;
reg  [1:0] a = 2'b00;
reg  [7:0] din = 8'h00;
reg        ncs = 1'b1, nwr = 1'b1, nrd = 1'b1;
wire [7:0] dout;
wire       busy, drq, intrq;

// ---------------------------------------------------------------- дисковод
reg        rawr_n = 1'b1, wprt_n = 1'b1, index_n = 1'b1;
reg        trk0_dead = 1'b0;                     // TR00 не приходит никогда
wire       hld, wg, wr_data, step, dir, tg43;
integer    pos = 3;                              // головка на дорожке
wire       tr00_n = trk0_dead ? 1'b1 : (pos != 0);
// HLT: одновибратор от HLD, как советует паспорт (с. 6): 1 мс после фронта HLD
reg        hlt = 1'b0;
always @(posedge hld) begin #1_000_000; if (hld) hlt = 1'b1; end
always @(negedge hld) hlt = 1'b0;

fdc_core #(.CLK_DIV(0)) dut (
    .clk(clk), .nres(nres),
    .a(a), .din(din), .dout(dout), .ncs(ncs), .nwr(nwr), .nrd(nrd),
    .busy(busy), .drq(drq), .intrq(intrq),
    .rawr_n(rawr_n), .wprt_n(wprt_n), .tr00_n(tr00_n), .index_n(index_n),
    .hrdy(hlt), .hld(hld), .wg(wg), .wr_data(wr_data), .step(step), .dir(dir),
    .tg43(tg43), .precomp(4'd0)
);

// ---------------------------------------------------------------- проверки
integer errors = 0, notes = 0;
`define CHECK(msg, cond) \
    if (cond) $display("   ok: %0s", msg); \
    else begin $display("   ПРОВАЛ: %0s", msg); errors = errors + 1; end
`define NOTE(msg, cond) \
    if (cond) $display("   ok: %0s", msg); \
    else begin $display("   пока нет: %0s", msg); notes = notes + 1; end

// ---------------------------------------------------------------- шина: задачи
// Строб около микросекунды, как у живой машины.
task wr(input [1:0] adr, input [7:0] v);
begin
    @(negedge clk); a = adr; din = v; ncs = 0; nwr = 0;
    repeat (16) @(negedge clk);
    nwr = 1; ncs = 1;
    repeat (8) @(negedge clk);
end
endtask
task rd(input [1:0] adr, output [7:0] v);
begin
    @(negedge clk); a = adr; ncs = 0; nrd = 0;
    repeat (16) @(negedge clk);
    v = dout;
    nrd = 1; ncs = 1;
    repeat (8) @(negedge clk);
end
endtask

reg [7:0] st, v8;
// ждать конца команды по INTRQ (а не опросом состояния: опрос сам гасит INTRQ)
task wait_intrq(input integer limit_ms, output integer took_us);
    realtime t0;
begin
    t0 = $realtime;
    while (!intrq && ($realtime - t0) < limit_ms * 1.0e6) #1000;
    took_us = ($realtime - t0) / 1000.0;
end
endtask
task cmd(input [7:0] c); begin wr(2'b00, c); end endtask

// ---------------------------------------------------------------- носитель
localparam integer CELL_NS = 2000;
localparam integer NSEC = 3;
integer id_trk_off = 0, id_side = 0;
integer bad_id_crc = 0, bad_dt_crc = 0, deleted_sec = 0, no_data_sec = 0;
reg [15:0] crc;
reg        prev_bit = 1'b0;

function [7:0] sec_byte(input integer t, input integer s, input integer i);
    sec_byte = (t * 7 + s * 31 + i * 3 + (i >> 4)) & 8'hFF;
endfunction

task emit_cell(input v);
begin
    if (v) begin #(CELL_NS/2 - 100); rawr_n = 1'b0; #200; rawr_n = 1'b1; #(CELL_NS/2 - 100); end
    else #CELL_NS;
end
endtask
task crc_byte(input [7:0] b);
    integer k;
begin
    crc = crc ^ {b, 8'h00};
    for (k = 0; k < 8; k = k + 1) crc = crc[15] ? {crc[14:0],1'b0} ^ 16'h1021 : {crc[14:0],1'b0};
end
endtask
task emit_byte(input [7:0] b);
    integer k; reg d;
begin
    for (k = 7; k >= 0; k = k - 1) begin
        d = b[k]; emit_cell(~(prev_bit | d)); emit_cell(d); prev_bit = d;
    end
    crc_byte(b);
end
endtask
task emit_a1;
    integer k; reg [15:0] pat;
begin
    pat = 16'h4489;
    for (k = 15; k >= 0; k = k - 1) emit_cell(pat[k]);
    prev_bit = 1'b1; crc_byte(8'hA1);
end
endtask
task emit_n(input [7:0] b, input integer n);
    integer k;
begin for (k = 0; k < n; k = k + 1) emit_byte(b); end
endtask
task emit_crc(input bad);
    reg [15:0] c;
begin
    c = crc ^ (bad ? 16'h0001 : 16'h0000);
    emit_byte(c[15:8]); emit_byte(c[7:0]);
end
endtask

integer n_index = 0;
task emit_rev;
    integer s, k, t;
begin
    index_n = 1'b0; n_index = n_index + 1; #200_000; index_n = 1'b1;
    t = pos;
    emit_n(8'h4E, 16);
    for (s = 1; s <= NSEC; s = s + 1) begin
        emit_n(8'h00, 12);
        crc = 16'hFFFF;
        emit_a1; emit_a1; emit_a1;
        emit_byte(8'hFE);
        emit_byte(t + id_trk_off); emit_byte(id_side); emit_byte(s); emit_byte(8'h01);
        emit_crc(bad_id_crc == s);
        emit_n(8'h4E, 22);
        if (no_data_sec == s) emit_n(8'h4E, 12 + 4 + 256 + 2);
        else begin
            emit_n(8'h00, 12);
            crc = 16'hFFFF;
            emit_a1; emit_a1; emit_a1;
            emit_byte(deleted_sec == s ? 8'hF8 : 8'hFB);
            for (k = 0; k < 256; k = k + 1) emit_byte(sec_byte(t, s, k));
            emit_crc(bad_dt_crc == s);
        end
        emit_n(8'h4E, 24);
    end
    emit_n(8'h4E, 20);
end
endtask
initial begin #300_000; forever emit_rev; end

// головка шагает по фронту STEP; DIRC = 1 -- к центру
integer   n_steps = 0, n_steps_in = 0, n_steps_out = 0;
realtime  step_t [0:400];
realtime  step_w;            // ширина последнего импульса шага
realtime  dir_chg_t = 0.0;   // когда последний раз менялось направление
realtime  dir_setup_min = 1.0e12;
reg       dir_d = 1'b0;
always @(dir) dir_chg_t = $realtime;
always @(posedge step) begin
    if (n_steps < 400) step_t[n_steps] = $realtime;
    if ($realtime - dir_chg_t < dir_setup_min) dir_setup_min = $realtime - dir_chg_t;
    if ($realtime - dir_chg_t < 24_000.0)
        $display("   [шаг %0d в %0.3f мс: направление %0d сменилось за %0.1f мкс, команда %02h, стадия %0d]",
                 n_steps, $realtime / 1.0e6, dir, ($realtime - dir_chg_t) / 1000.0, dut.U14.rREG_CMD, dut.U14.rSTAGE);
    n_steps = n_steps + 1;
    if (dir) begin n_steps_in = n_steps_in + 1; if (pos < 83) pos = pos + 1; end
    else     begin n_steps_out = n_steps_out + 1; if (pos > 0) pos = pos - 1; end
end
realtime step_rise;
always @(posedge step) step_rise = $realtime;
always @(negedge step) step_w = $realtime - step_rise;

// запись: перепады WD, пока взведён WG
integer   n_wr = 0, wg_up = 0;
real      wr_t [0:40000];
realtime  wg_rise_t = 0.0, wg_fall_t = 0.0;
always @(negedge wg) wg_fall_t = $realtime;
reg       wd_d = 1'b1, wg_d = 1'b0;
always @(posedge clk) begin
    wg_d <= wg;
    if (wg && !wg_d) begin wg_up = wg_up + 1; n_wr = 0; wg_rise_t = $realtime; end
    wd_d <= wr_data;
    if (wg && wd_d && !wr_data && n_wr < 40000) begin wr_t[n_wr] = $realtime; n_wr = n_wr + 1; end
end
reg [0:90000] cells;
integer       n_cells, sync_at, n_dec;
reg [7:0]     dec [0:1200];
task decode_written;
    integer i, k, gap, p, b; reg [15:0] w;
begin
    cells[0] = 1'b1; n_cells = 1;
    for (i = 1; i < n_wr; i = i + 1) begin
        gap = (wr_t[i] - wr_t[i-1]) / CELL_NS;
        if (gap < 1) gap = 1;
        for (k = 1; k < gap; k = k + 1) begin cells[n_cells] = 1'b0; n_cells = n_cells + 1; end
        cells[n_cells] = 1'b1; n_cells = n_cells + 1;
    end
    sync_at = -1;
    for (i = 0; i + 16 <= n_cells && sync_at < 0; i = i + 1) begin
        w = 16'h0;
        for (k = 0; k < 16; k = k + 1) w = {w[14:0], cells[i+k]};
        if (w == 16'h4489) sync_at = i;
    end
    n_dec = 0;
    if (sync_at >= 0) begin
        p = sync_at;
        while (p + 16 <= n_cells && n_dec < 1200) begin
            b = 0;
            for (k = 0; k < 8; k = k + 1) b = (b << 1) | cells[p + 2*k + 1];
            dec[n_dec] = b[7:0]; n_dec = n_dec + 1; p = p + 16;
        end
    end
end
endtask
function [15:0] crc_dec(input integer from, input integer n);
    integer i, q; reg [15:0] c;
begin
    c = 16'hFFFF;
    for (i = from; i < from + n; i = i + 1) begin
        c = c ^ {dec[i], 8'h00};
        for (q = 0; q < 8; q = q + 1) c = c[15] ? {c[14:0],1'b0} ^ 16'h1021 : {c[14:0],1'b0};
    end
    crc_dec = c;
end
endfunction

// чтение сектора(ов) с обслуживанием DRQ; skip -- сколько DRQ подряд не
// обслужить начиная с байта skip_at (для потери данных)
reg [7:0] got [0:2047];
integer   n_got;
realtime  t_cmd, t_first_drq;
task read_cmd(input [7:0] c, input integer limit_ms, input integer skip_at, input integer skip);
    realtime t0; integer skipped;
begin
    n_got = 0; skipped = 0; t_first_drq = 0.0;
    t_cmd = $realtime;
    cmd(c);
    t0 = $realtime;
    while (!intrq && ($realtime - t0) < limit_ms * 1.0e6) begin
        @(posedge clk);
        if (drq) begin
            if (t_first_drq == 0.0) t_first_drq = $realtime;
            if (n_got == skip_at && skipped < skip) begin
                // пропустить: дождаться, пока DRQ упадёт сам при следующем байте
                skipped = skipped + 1;
                #40_000;
            end else begin
                rd(2'b11, v8); if (n_got < 2048) got[n_got] = v8; n_got = n_got + 1;
            end
        end
    end
end
endtask

// запись сектора: подать n байт по DRQ; zero_at -- на этом байте DRQ не обслужить
task write_cmd(input [7:0] c, input integer n, input integer limit_ms, input integer miss_at);
    realtime t0; integer k, sent;
begin
    sent = 0;
    cmd(c);
    t0 = $realtime;
    while (!intrq && ($realtime - t0) < limit_ms * 1.0e6) begin
        @(posedge clk);
        if (drq && sent < n && miss_at != -2) begin
            if (sent == miss_at) begin #40_000; sent = sent + 1; end
            else begin wr(2'b11, sec_byte(40, 1, sent)); sent = sent + 1; end
        end
    end
    // INTRQ ядро даёт, сняв свой WG, а наружу WG с последним байтом (FF за CRC)
    // гаснет на несколько ячеек позже (mfm_wr: дописать байт и две ячейки выдачи)
    if (wg) wait (!wg);
    #20_000;
end
endtask

realtime t_start, t_end;
integer  took, k, i0, n_idx0, bad;

initial begin
    $display("Сверка ядра ВГ93 (fdc_core) с паспортом FD179X-01");
    #60_000; nres = 1'b1;                           // сброс не короче TMR = 50 мкс (Miscellaneous Timing)
    wait_intrq(2000, took);                         // самовосстановление по сбросу
    rd(2'b00, st);

    // ================================================================ 1
    $display("");
    $display("== 1. Регистры и INTRQ (с. 5-6)");
    wr(2'b01, 8'h2A); rd(2'b01, v8);
        `CHECK("регистр дорожки пишется и читается", v8 === 8'h2A);
    wr(2'b10, 8'h05); rd(2'b10, v8);
        `CHECK("регистр сектора пишется и читается", v8 === 8'h05);
    wr(2'b11, 8'h5A); rd(2'b11, v8);
        `CHECK("регистр данных пишется и читается", v8 === 8'h5A);
    wr(2'b01, 8'd3);
    cmd(8'h03);                                     // Restore, 30 мс
    wait_intrq(2000, took);
        `CHECK("по концу команды INTRQ взведён", intrq === 1'b1);
    rd(2'b00, st);
    #1000;
        `CHECK("чтение состояния гасит INTRQ", intrq === 1'b0);
    cmd(8'h03); wait_intrq(2000, took);
        `CHECK("INTRQ снова взведён", intrq === 1'b1);
    wr(2'b01, 8'd0); wr(2'b11, 8'd3);
    cmd(8'h13);                                     // новая команда, Seek на 3 по 30 мс
    #1000;
        `CHECK("запись команды гасит INTRQ", intrq === 1'b0);
    wait_intrq(2000, took); rd(2'b00, st);
    cmd(8'h03); wait_intrq(2000, took); rd(2'b00, st);   // обратно на 0

    // ================================================================ 2
    $display("");
    $display("== 2. Состояние типа I (таблица 6, с. 14)");
    rd(2'b00, st);
        `CHECK("S2 ДОРОЖКА 0 -- копия TR00 (головка на нулевой)", st[2] === 1'b1 && pos == 0);
    wprt_n = 1'b0; #2000; rd(2'b00, st);
        `CHECK("S6 ЗАЩИТА -- копия WPRT", st[6] === 1'b1);
    wprt_n = 1'b1; #2000; rd(2'b00, st);
        `CHECK("S6 снимается вместе с WPRT", st[6] === 1'b0);
    wait (index_n == 1'b0); #50_000; rd(2'b00, st);
        `CHECK("S1 ИНДЕКС -- копия IP во время импульса", st[1] === 1'b1);
    wait (index_n == 1'b1); #300_000; rd(2'b00, st);
        `CHECK("S1 снят вне импульса индекса", st[1] === 1'b0);
    cmd(8'h08); wait_intrq(2000, took); #2_000_000; rd(2'b00, st);
        `CHECK("Restore с h=1: HLD взведён", hld === 1'b1);
        `CHECK("S5 ГОЛОВКА ЗАГРУЖЕНА = HLD и HLT", st[5] === 1'b1);
    cmd(8'h00); wait_intrq(2000, took); #2000; rd(2'b00, st);
        `CHECK("Restore с h=0, V=0: HLD снят", hld === 1'b0);
        `CHECK("S5 снят вместе с HLD", st[5] === 1'b0);
        `NOTE("S7 НЕ ГОТОВ -- копия READY (у ядра входа READY нет, бит зашит; готовность подставляет внешняя обвязка)", st[7] === 1'b0);

    // ================================================================ 3
    $display("");
    $display("== 3. Темпы шагов на CLK = 1 МГц: 6, 12, 20, 30 мс (таблица 1, с. 6)");
    for (k = 0; k < 4; k = k + 1) begin : rates
        realtime dt; integer exp_ms;
        exp_ms = (k == 0) ? 6 : (k == 1) ? 12 : (k == 2) ? 20 : 30;
        wr(2'b01, 8'd0); wr(2'b11, 8'd2);
        i0 = n_steps;
        cmd(8'h10 | k[1:0]);                        // Seek на 2, без проверки
        wait_intrq(500, took); rd(2'b00, st);
        dt = (step_t[i0 + 1] - step_t[i0]) / 1.0e6;
        $display("   r=%0d: между шагами %0.2f мс, ждём %0d", k, dt, exp_ms);
        `CHECK("темп шагов по таблице 1 (на 1 МГц), ±2 %", dt > exp_ms * 0.98 && dt < exp_ms * 1.02);
        wr(2'b01, 8'd2); wr(2'b11, 8'd0); cmd(8'h10); wait_intrq(500, took); rd(2'b00, st);
    end

    // ================================================================ 4
    $display("");
    $display("== 4. Импульс шага и установка направления (Miscellaneous Timing)");
    $display("   ширина импульса шага %0.2f мкс, направление установлено за %0.1f мкс до шага",
             step_w / 1000.0, dir_setup_min / 1000.0);
        `CHECK("импульс шага TSTP не короче 4 мкс (2 мкс на 2 МГц, на 1 МГц вдвое)", step_w >= 3990.0);
        `CHECK("импульс шага не короче 0,8 мкс -- минимум у 3,5-дюймовых и пятидюймовых приводов", step_w >= 800.0);
        `CHECK("направление установлено не позже чем за 24 мкс до шага (TDIR 12 мкс, на 1 МГц вдвое)",
               dir_setup_min >= 24_000.0);

    // ================================================================ 5
    $display("");
    $display("== 5. Step, Step-In, Step-Out и флаг u (с. 9-10)");
    wr(2'b01, 8'd0);                                // головка на 0
    cmd(8'h50); wait_intrq(500, took); rd(2'b00, st); rd(2'b01, v8);
        `CHECK("Step-In u=1: головка на 1, регистр дорожки 1", pos == 1 && v8 === 8'd1);
    cmd(8'h40); wait_intrq(500, took); rd(2'b00, st); rd(2'b01, v8);
        `CHECK("Step-In u=0: головка на 2, регистр дорожки не тронут (1)", pos == 2 && v8 === 8'd1);
    cmd(8'h30); wait_intrq(500, took); rd(2'b00, st); rd(2'b01, v8);
        `CHECK("Step u=1 -- в сторону прошлого шага (к центру): головка 3, регистр 2", pos == 3 && v8 === 8'd2);
    cmd(8'h70); wait_intrq(500, took); rd(2'b00, st); rd(2'b01, v8);
        `CHECK("Step-Out u=1: головка 2, регистр 1", pos == 2 && v8 === 8'd1);
    cmd(8'h30); wait_intrq(500, took); rd(2'b00, st); rd(2'b01, v8);
        `CHECK("Step после Step-Out -- наружу: головка 1, регистр 0", pos == 1 && v8 === 8'd0);
    cmd(8'h60); wait_intrq(500, took); rd(2'b00, st); rd(2'b01, v8);
        `CHECK("Step-Out u=0: головка 0, регистр не тронут (0)", pos == 0 && v8 === 8'd0);

    // ================================================================ 6
    $display("");
    $display("== 6. Проверка дорожки, V=1 (с. 6, 8)");
    wr(2'b01, 8'd0); wr(2'b11, 8'd2);
    i0 = n_steps;
    cmd(8'h1C);                                     // Seek на 2, h=1, V=1, 6 мс
    wait_intrq(2000, took); t_end = $realtime; rd(2'b00, st);
    $display("   Seek с проверкой: %0d мкс, состояние %02h", took, st);
        `CHECK("дорожка совпала: без ошибки поиска (S4) и CRC (S3)", st[4] === 1'b0 && st[3] === 1'b0);
        `CHECK("проверка началась не раньше 30 мс после последнего шага (15 мс на 2 МГц)",
               (t_end - step_t[n_steps - 1]) >= 30.0e6);
    id_trk_off = 1;                                 // в поле адреса -- дорожка 3
    wr(2'b01, 8'd2); wr(2'b11, 8'd2);
    n_idx0 = n_index;
    cmd(8'h1C); wait_intrq(3000, took); rd(2'b00, st);
    $display("   поле адреса с чужой дорожкой: %0d мкс, %0d индексов, состояние %02h", took, n_index - n_idx0, st);
        `CHECK("номер дорожки не тот -- ошибка поиска S4", st[4] === 1'b1);
        `CHECK("ошибка поиска -- не дольше 5 оборотов (с. 6: within 5 revolutions)", (n_index - n_idx0) <= 6);
    id_trk_off = 0;
    bad_id_crc = 1;
    $display("   (CRC порчена только у сектора 1, остальные целые -- проверка обязана пройти по ним)");
    wr(2'b01, 8'd2); wr(2'b11, 8'd2);
    cmd(8'h1C); wait_intrq(3000, took); rd(2'b00, st);
        `CHECK("CRC одного поля плохая, следующее хорошее -- проверка прошла (S4 снят)", st[4] === 1'b0);
    bad_id_crc = 0;

    // ================================================================ 7
    $display("");
    $display("== 7. Головка: h, автоснятие через 15 оборотов (с. 6, 8)");
    cmd(8'h08); wait_intrq(500, took); rd(2'b00, st);
        `CHECK("h=1: HLD в начале команды", hld === 1'b1);
    n_idx0 = n_index;
    while (hld && (n_index - n_idx0) < 20) #1_000_000;
    $display("   HLD снят после %0d индексов простоя", n_index - n_idx0);
        `CHECK("простой 15 оборотов -- HLD снят сам", hld === 1'b0 && (n_index - n_idx0) >= 15 && (n_index - n_idx0) <= 16);

    // ================================================================ 8
    $display("");
    $display("== 8. Read Sector: данные, тип записи S5, E (с. 10-11)");
    wr(2'b01, 8'd0); wr(2'b11, 8'd2); cmd(8'h10); wait_intrq(500, took); rd(2'b00, st);
    wr(2'b10, 8'd2);
    read_cmd(8'h80, 1000, -1, 0); rd(2'b00, st);
    bad = 0; for (k = 0; k < 256; k = k + 1) if (got[k] !== sec_byte(2, 2, k)) bad = bad + 1;
    $display("   сектор 2: %0d байт, расхождений %0d, состояние %02h", n_got, bad, st);
        `CHECK("256 байт сектора 2, все верны, состояние чистое", n_got == 256 && bad == 0 && st[5:1] === 5'b00000);
    deleted_sec = 2;
    read_cmd(8'h80, 1000, -1, 0); rd(2'b00, st);
        `CHECK("метка F8: S5 ТИП ЗАПИСИ = 1 (удалённые данные)", st[5] === 1'b1 && n_got == 256);
    deleted_sec = 0;
    // E: задержка 15 мс (на 1 МГц 30) до опроса HLT. Команду даём сразу за
    // индексом: сектор 1 идёт через ~1 мс. Без E он читается в этом же обороте,
    // с E -- пропускается, и первый DRQ приходит не раньше 30 мс.
    wr(2'b10, 8'd1);
    wait (index_n == 1'b0); wait (index_n == 1'b1);
    read_cmd(8'h80, 1000, -1, 0); rd(2'b00, st);
    $display("   E=0: первый DRQ через %0.1f мс", (t_first_drq - t_cmd) / 1.0e6);
        `CHECK("E=0: сектор 1 прочитан в том же обороте (DRQ раньше 30 мс)",
               n_got == 256 && (t_first_drq - t_cmd) < 30.0e6);
    wait (index_n == 1'b0); wait (index_n == 1'b1);
    read_cmd(8'h84, 1000, -1, 0); rd(2'b00, st);
    $display("   E=1: первый DRQ через %0.1f мс", (t_first_drq - t_cmd) / 1.0e6);
        `CHECK("E=1: задержка 30 мс до поиска (15 мс на 2 МГц) -- DRQ не раньше 30 мс",
               n_got == 256 && (t_first_drq - t_cmd) >= 30.0e6);

    // ================================================================ 9
    $display("");
    $display("== 9. Ошибки чтения: CRC, сектор не найден, потеря данных (с. 10-11)");
    bad_dt_crc = 2; wr(2'b10, 8'd2);
    read_cmd(8'h80, 1000, -1, 0); rd(2'b00, st);
        `CHECK("CRC поля данных плохая -- S3 CRC, S4 снят", st[3] === 1'b1 && st[4] === 1'b0);
    bad_dt_crc = 0;
    wr(2'b10, 8'd9);
    n_idx0 = n_index;
    read_cmd(8'h80, 3000, -1, 0); rd(2'b00, st);
    $display("   сектора 9 нет: %0d индексов до конца, состояние %02h", n_index - n_idx0, st);
        `CHECK("сектор не найден -- S4 RNF", st[4] === 1'b1 && n_got == 0);
        `CHECK("S3 CRC прошлой команды сброшен новой (поля адреса целые)", st[3] === 1'b0);
        `CHECK("ищет не дольше 5 оборотов (паспорт: 4 оборота / 5 индексов)", (n_index - n_idx0) >= 4 && (n_index - n_idx0) <= 6);
    no_data_sec = 3; wr(2'b10, 8'd3);
    n_idx0 = n_index;
    read_cmd(8'h80, 3000, -1, 0); rd(2'b00, st);
    $display("   сектор 3 без поля данных: принято %0d байт, первые %02h %02h %02h, состояние %02h",
             n_got, got[0], got[1], got[2], st);
        `CHECK("поле адреса есть, метки данных за 43 байта нет -- S4 RNF", st[4] === 1'b1 && n_got == 0);
        `CHECK("и сразу, в том же обороте, а не через 5", (n_index - n_idx0) <= 2);
    no_data_sec = 0;
    wr(2'b10, 8'd1);
    read_cmd(8'h80, 1000, 100, 3); rd(2'b00, st);
    $display("   пропущено 3 DRQ: принято %0d байт, состояние %02h", n_got, st);
        `CHECK("не забрали байт вовремя -- S2 ПОТЕРЯ ДАННЫХ", st[2] === 1'b1);
        `CHECK("чтение дошло до конца сектора (256 - 3 пропущенных)", n_got == 253);

    // ================================================================ 10
    $display("");
    $display("== 10. Сравнение стороны C/S, несколько секторов m (с. 10)");
    wr(2'b10, 8'd1);
    read_cmd(8'h82, 1000, -1, 0); rd(2'b00, st);   // C=1, S=0
        `CHECK("C=1, S=0, в поле адреса сторона 0 -- найден", n_got == 256 && st[4] === 1'b0);
    read_cmd(8'h8A, 3000, -1, 0); rd(2'b00, st);   // C=1, S=1
        `CHECK("C=1, S=1, а в поле адреса 0 -- сектор не найден", st[4] === 1'b1 && n_got == 0);
    id_side = 1;
    read_cmd(8'h80, 1000, -1, 0); rd(2'b00, st);   // C=0
        `CHECK("C=0 -- сторона не сравнивается (в поле 1, найден)", n_got == 256 && st[4] === 1'b0);
    id_side = 0;
    wr(2'b10, 8'd1);
    read_cmd(8'h90, 3000, -1, 0); rd(2'b00, st); rd(2'b10, v8);
    bad = 0;
    for (k = 0; k < 768 && k < n_got; k = k + 1) if (got[k] !== sec_byte(2, 1 + k / 256, k % 256)) bad = bad + 1;
    $display("   m=1 с сектора 1: %0d байт, расхождений %0d, регистр сектора %0d, состояние %02h", n_got, bad, v8, st);
        `CHECK("m=1: прочитаны сектора 1, 2, 3 подряд, все байты верны", n_got == 768 && bad == 0);
        `CHECK("регистр сектора ушёл за последний (4), S4 RNF", v8 === 8'd4 && st[4] === 1'b1);

    // ================================================================ 11
    $display("");
    $display("== 11. Write Sector: метка a0, WG, потеря данных, защита (с. 11)");
    wr(2'b10, 8'd2);
    write_cmd(8'hA1, 256, 1000, -1); rd(2'b00, st);  // a0 = 1 -- F8
    decode_written;
    $display("   записано: синхро на %0d, байт %0d, метка %02h, состояние %02h", sync_at, n_dec, dec[3], st);
        `CHECK("a0=1 -- записана метка F8 (удалённые данные)", n_dec > 4 && dec[3] === 8'hF8);
        `CHECK("CRC поля данных верна", n_dec > 261 && {dec[260], dec[261]} === crc_dec(0, 260));
    $display("   за CRC записано: %02h %02h %02h (перепадов %0d, ячеек %0d от синхро %0d, последний перепад за %0.1f мкс до снятия WG)",
             dec[262], dec[263], dec[264], n_wr, n_cells, sync_at, (wg_fall_t - wr_t[n_wr-1]) / 1000.0);
        `CHECK("за CRC -- байт FF", n_dec > 262 && dec[262] === 8'hFF);
    write_cmd(8'hA0, 256, 1000, 50); rd(2'b00, st);  // байт 50 не подан
    decode_written;
    $display("   пропуск байта 50: состояние %02h, байт 50 записан как %02h", st, dec[4 + 50]);
        `CHECK("не подали байт -- S2 ПОТЕРЯ ДАННЫХ, команда не прервана", st[2] === 1'b1 && n_dec > 262);
        `CHECK("вместо него записан ноль", dec[4 + 50] === 8'h00);
    begin : first_miss
        integer wg0; wg0 = wg_up;
        write_cmd(8'hA0, 256, 1000, -2); rd(2'b00, st);   // DRQ не обслуживать вовсе
        $display("   DRQ не обслужен: состояние %02h", st);
        `CHECK("DRQ не обслужен до 22-го байта -- команда прервана с ПОТЕРЕЙ ДАННЫХ, WG не взводился",
               st[2] === 1'b1 && wg_up == wg0);
    end
    wprt_n = 1'b0;
    begin : wp
        integer wg0; wg0 = wg_up;
        write_cmd(8'hA0, 256, 1000, -1); rd(2'b00, st);
        `CHECK("защита записи -- команда сразу прервана, S6, WG не взводился", st[6] === 1'b1 && wg_up == wg0);
    end
    wprt_n = 1'b1;

    // ================================================================ 12
    $display("");
    $display("== 12. Read Address: регистр сектора, CRC (с. 13)");
    wr(2'b10, 8'h77);
    read_cmd(8'hC0, 1000, -1, 0); rd(2'b00, st); rd(2'b10, v8);
    $display("   поле: %02h %02h %02h %02h, регистр сектора %02h, состояние %02h", got[0], got[1], got[2], got[3], v8, st);
        `CHECK("шесть байт поля адреса", n_got == 6);
        `CHECK("номер дорожки из поля адреса записан в регистр сектора", v8 === got[0]);
    bad_id_crc = 1; bad_dt_crc = 0;
    begin : ra_crc
        integer tries; tries = 0;
        st = 8'h00;
        while (tries < 4 && !(n_got == 6 && got[2] === 8'd1)) begin
            read_cmd(8'hC0, 1000, -1, 0); rd(2'b00, st); tries = tries + 1;
        end
        `CHECK("поле адреса с плохой CRC -- S3 CRC", n_got == 6 && got[2] === 8'd1 && st[3] === 1'b1);
    end
    bad_id_crc = 0;

    // ================================================================ 13
    $display("");
    $display("== 13. Write Track: DRQ сразу, без байта к индексу -- прервать (с. 13)");
    begin : wt
        integer wg0; wg0 = wg_up;
        cmd(8'hF0); #20_000;
        `CHECK("DRQ взведён сразу по команде", drq === 1'b1);
        wait_intrq(1000, took); rd(2'b00, st);
        $display("   Write Track без байта: %0d мкс, состояние %02h", took, st);
        `CHECK("регистр данных не загружен к индексу -- конец, ПОТЕРЯ ДАННЫХ, WG не взводился",
               st[2] === 1'b1 && st[0] === 1'b0 && wg_up == wg0);
    end
    // первый байт -- не сразу, а через 2 мс (до индекса): по паспорту запись идёт
    begin : wt_late
        integer wg0, nb; wg0 = wg_up; nb = 0;
        wait (index_n == 1'b0); wait (index_n == 1'b1);   // индекс только что прошёл
        cmd(8'hF0); #2_000_000;
        wr(2'b11, 8'h4E); nb = 1;
        while (!intrq && nb < 4000) begin
            @(posedge clk);
            if (drq) begin wr(2'b11, 8'h4E); nb = nb + 1; end
        end
        if (wg) wait (!wg);
        rd(2'b00, st);
        $display("   первый байт через 2 мс: подано %0d байт, WG взводился %0d раз, состояние %02h", nb, wg_up - wg0, st);
        `CHECK("первый байт до индекса -- форматирование пошло (WG), без потери данных",
               wg_up == wg0 + 1 && st[2] === 1'b0 && nb > 1000);
    end

    // ================================================================ 14
    $display("");
    $display("== 14. Force Interrupt (с. 13-14)");
    wr(2'b01, 8'd2); wr(2'b11, 8'd40);
    cmd(8'h13);                                     // Seek на 40, 30 мс на шаг
    #100_000_000;
    cmd(8'hD0);                                     // прервать без INTRQ
    #1_000_000; rd(2'b00, st);
        `CHECK("D0: команда прервана, занятость снята", st[0] === 1'b0);
    k = n_steps; #100_000_000;
        `CHECK("D0: шаги прекратились", n_steps == k);
    // INTRQ после D0 -- по паспорту не взводится; rd выше его всё равно гасит,
    // поэтому смотрим отдельно
    cmd(8'h13); #100_000_000; cmd(8'hD0); #1_000_000;
        `CHECK("D0: INTRQ не взводится", intrq === 1'b0);
    rd(2'b00, st);
    cmd(8'hD4);                                     // по каждому индексу
    wait (index_n == 1'b1); wait (index_n == 1'b0); #100_000;
        `CHECK("D4: INTRQ по импульсу индекса", intrq === 1'b1);
    rd(2'b00, st);
    cmd(8'hD0); #1000; rd(2'b00, st);
    // команда, поданная при занятости (не Force Interrupt), не принимается
    cmd(8'h00); wait_intrq(2000, took); rd(2'b00, st);
    wr(2'b01, 8'd0); wr(2'b11, 8'd30);
    cmd(8'h13); #50_000_000;
    wr(2'b10, 8'd1); cmd(8'h80);                    // Read Sector посреди Seek
    wait_intrq(3000, took); rd(2'b00, st); rd(2'b01, v8);
    $display("   после Read Sector посреди Seek: головка %0d, регистр дорожки %0d, состояние %02h, %0d мкс", pos, v8, st, took);
        // Паспорт (с. 7) требует это от программы («Command words should only be
        // loaded ... when the Busy status bit is off»), а что делает сама
        // микросхема -- не говорит. Ядро принимает команду и прерывает текущую.
        `NOTE("команда при занятости проигнорирована (поведение микросхемы паспортом не задано)", v8 === 8'd30 && pos == 30);

    // ================================================================ 15
    $display("");
    $display("== 15. Restore без TR00: 255 шагов и ошибка поиска (с. 9)");
    trk0_dead = 1'b1;
    i0 = n_steps;
    cmd(8'h00); wait_intrq(3000, took); rd(2'b00, st);
    $display("   шагов %0d, состояние %02h", n_steps - i0, st);
        `CHECK("255 шагов без TR00 -- конец с S4 ОШИБКА ПОИСКА", (n_steps - i0) == 255 && st[4] === 1'b1);
    trk0_dead = 1'b0;
    cmd(8'h00); wait_intrq(3000, took); rd(2'b00, st);
        `CHECK("следующий Restore сбрасывает S4 (паспорт: «updated or cleared for the new command»)", st[4] === 1'b0);

    $display("");
    if (errors == 0) $display("ВСЕ ПРОВЕРКИ ПРОЙДЕНЫ (отличий от паспорта, известных и записанных: %0d)", notes);
    else             $display("ПРОВАЛОВ: %0d, отличий: %0d", errors, notes);
    $finish;
end

endmodule

`default_nettype wire
