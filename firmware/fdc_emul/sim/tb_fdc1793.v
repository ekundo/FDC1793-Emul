// Стенд ядра ВГ93 (rtl/fdc_emul.v и rtl/fdc) против даташита WD1793.
//
// Писался под ядро как есть, без правок: смысл прогона — выяснить, что оно
// делает на самом деле, а не что написано в его README. Проверки сверены с
// таблицей команд и с регистром состояния WD1793. Правки ядра, найденные по
// ходу, перечислены в README.md в корне. Запуск -- sim/run.sh.
//
// ТАКТ. У ядра внутри clk_16 = clk/2, а константы задержек считаны от 16 МГц:
// DELAY30 = 234 при предделителе 2048 даёт ровно 30 мс. Значит на clk надо
// 32 МГц, и здесь подаётся именно столько.
//
// УРОВНИ. Ядро инвертирует входы привода само (iTR00(~FDC_TR00) и так далее),
// то есть ждёт НИЗКОАКТИВНЫЕ уровни, как у живого дисковода. Модель привода
// ниже такие и выдаёт.
//
// Привод моделируется грубо: счётчик дорожки, ДОР 0 и индекс. Ни MFM, ни
// секторов здесь нет — это первый этап, команды позиционирования (тип I).
`timescale 1ns/1ps
`default_nettype none

module tb_fdc1793;

// 32 МГц ТОЧНО. Округление полупериода до 15 нс давало 33,3 МГц, то есть
// ядро бежало на 4,2 % быстрее носителя: за 48 ячеек накапливалось две ячейки
// расхождения, и тройка синхромаркеров не собиралась никогда. Разрешения
// timescale (1 пс) на 15,625 хватает.
localparam real HALF = 15.625;

reg         clk = 0;
reg  [1:0]  a = 2'b00;
reg         nwr = 1, nrd = 1, ncs = 1, nres = 0;
reg  [7:0]  drive_data = 8'h00;
reg         drive_bus = 0;

wire [7:0]  data = drive_bus ? drive_data : 8'bz;
wire        drq, intrq, tr43, hld, wg, wdata_o, step, dir, wf_de;

integer errors = 0;

// ---- модель привода
reg  [7:0]  track = 8'd3;              // головка стоит на 3-й дорожке
reg         index_n = 1;
wire        trk00_n = (track != 0);
wire        wprt_n  = 1'b1;            // не защищена

integer steps = 0;
reg step_d = 0;
always @(posedge clk) begin
    step_d <= step;
    if (step && !step_d) begin         // фронт шага
        steps <= steps + 1;
        if (dir) begin if (track < 8'd79) track <= track + 1'b1; end
        else     begin if (track > 0)     track <= track - 1'b1; end
    end
end

// индекс выдаёт модель носителя ниже, вместе с потоком MFM



// ---------------------------------------------------------------- модель носителя
// Поток MFM на FDC_nRAWR: то, что головка реально снимает с дискеты.
//
// Кодирование MFM: каждый разряд данных превращается в ДВЕ ячейки, тактовую и
// информационную. Информационная равна самому разряду, тактовая равна единице
// только если и предыдущий разряд, и текущий нули. Единица в ячейке — это
// перепад намагниченности, то есть импульс на линии чтения.
//
// Синхромаркер A1 — это тот же A1, но с ВЫБИТЫМ тактовым разрядом, и в ячейках
// он даёт 16'h4489. Ни на что другое такая последовательность не похожа, по ней
// контроллер и ловит начало поля. Выбивание записано прямо константой: выводить
// его правилом нельзя, оно правило и нарушает.
//
// Темп: 250 кбит/с, значит разряд данных 4 мкс, ячейка 2 мкс.
localparam integer CELL_NS = 2000;
localparam [7:0]   TRK_ON_DISK = 8'd5, SIDE_ON_DISK = 8'd0, SEC_ON_DISK = 8'd1;

reg        nrawr = 1'b1;
reg        mark_win = 1'b0;
reg        prev_bit = 1'b0;
reg [15:0] crc;
reg [7:0]  sector_data [0:255];

integer gi;
initial for (gi = 0; gi < 256; gi = gi + 1) sector_data[gi] = gi[7:0];

// один перепад
// Перепад ставим в СЕРЕДИНУ ячейки, а не в начало: петля ФАПЧ ловит его окном
// вокруг середины (в таблице DPLL.hex поправки сидят на состояниях 16..19 из
// 32). С перепадом в начале петля соскальзывает — ловятся одиночные маркеры и
// пары, а тройки, по которым и опознаётся поле, не собираются ни разу.
task emit_cell(input v);
begin
    if (v) begin
        #(CELL_NS/2 - 100); nrawr = 1'b0; #200; nrawr = 1'b1; #(CELL_NS/2 - 100);
    end else #CELL_NS;
end
endtask

task crc_byte(input [7:0] b);
    integer k;
begin
    crc = crc ^ {b, 8'h00};
    for (k = 0; k < 8; k = k + 1)
        crc = crc[15] ? {crc[14:0],1'b0} ^ 16'h1021 : {crc[14:0],1'b0};
end
endtask

// обычный байт: тактовая ячейка по правилу, информационная — сам разряд
task emit_byte(input [7:0] b);
    integer k; reg d;
begin
    for (k = 7; k >= 0; k = k - 1) begin
        d = b[k];
        emit_cell(~(prev_bit | d));         // тактовая
        emit_cell(d);                       // информационная
        prev_bit = d;
    end
    crc_byte(b);
end
endtask

// синхромаркер A1 с выбитым тактом: 16 ячеек 4489 константой
task emit_a1;
    integer k; reg [15:0] pat;
begin
    pat = 16'h4489;
    for (k = 15; k >= 0; k = k - 1) emit_cell(pat[k]);
    prev_bit = 1'b1;                   // младший разряд A1 = 1
    crc_byte(8'hA1);
end
endtask

task emit_n(input [7:0] b, input integer n);
    integer k;
begin for (k = 0; k < n; k = k + 1) emit_byte(b); end
endtask

// одна «дорожка»: индекс, поле адреса, поле данных, промежуток.
// Оборот укорочен намеренно — контроллеру важны метки, а не настоящие 200 мс.
task emit_track;
    integer k;
begin
    index_n = 1'b0; #500_000; index_n = 1'b1;
    emit_n(8'h4E, 16);
    emit_n(8'h00, 12);
    crc = 16'hFFFF;
    mark_win = 1'b1;
    emit_a1; emit_a1; emit_a1;
    mark_win = 1'b0;
    emit_byte(8'hFE);                            // метка адреса
    emit_byte(TRK_ON_DISK); emit_byte(SIDE_ON_DISK);
    emit_byte(SEC_ON_DISK); emit_byte(8'h01);    // 01 = 256 байт
    begin : idcrc
        reg [15:0] c; c = crc;
        emit_byte(c[15:8]); emit_byte(c[7:0]);
    end
    emit_n(8'h4E, 22);
    emit_n(8'h00, 12);
    crc = 16'hFFFF;
    emit_a1; emit_a1; emit_a1;
    emit_byte(8'hFB);                            // метка данных
    for (k = 0; k < 256; k = k + 1) emit_byte(sector_data[k]);
    begin : dtcrc
        reg [15:0] c; c = crc;
        emit_byte(c[15:8]); emit_byte(c[7:0]);
    end
    emit_n(8'h4E, 40);
end
endtask

initial begin
    #300_000;                          // дать сбросу пройти
    forever emit_track;
end

fdc_emul dut (
    .clk(clk), .FDC_CLK(1'b0),
    .FDC_A(a), .FDC_DATA(data), .FDC_nWR(nwr), .FDC_nRD(nrd),
    .nOE_245(), .DIR_245(),
    .FDC_nCS(ncs), .FDC_nRES(nres), .FDC_nDDEN(1'b0),
    .FDC_RDY(1'b1), .FDC_WF_DE(wf_de), .FDC_HRDY(hld),
    .FDC_DRQ(drq), .FDC_INTRQ(intrq), .FDC_TR43(tr43),
    .FDC_RCLK(1'b0), .FDC_nRAWR(nrawr),
    .FDC_HLD(hld), .FDC_RSTB(), .FDC_SL(), .FDC_SR(),
    .FDC_WPRT(wprt_n), .FDC_TR00(trk00_n), .FDC_INDEX(index_n),
    .FDC_WG(wg), .FDC_WR_DATA(wdata_o), .FDC_STEP(step), .FDC_DIR(dir)
);

always #(HALF) clk = ~clk;

// ---- шина процессора
task wr(input [1:0] adr, input [7:0] v);
begin
    // Строб держим около микросекунды, как живая машина: ядро защёлкивает по
    // rWR_EN & rWR_EN0, то есть по ДВУМ подряд выборкам clk_16, и короткий
    // импульс оно просто не заметит.
    @(negedge clk); a = adr; drive_data = v; drive_bus = 1; ncs = 0; nwr = 0;
    repeat (32) @(negedge clk);
    nwr = 1; ncs = 1; drive_bus = 0;
    repeat (16) @(negedge clk);
end
endtask

task rd(input [1:0] adr, output [7:0] v);
begin
    @(negedge clk); a = adr; ncs = 0; nrd = 0;
    repeat (32) @(negedge clk);
    v = data;
    nrd = 1; ncs = 1;
    repeat (16) @(negedge clk);
end
endtask

// Строку НЕ передаём через порт задачи: iverilog корёжит кириллицу в UTF-8,
// когда она едет через reg-вектор. Макрос оставляет её в самом $display.
// То же, что CHECK, но без приговора: для незаконченных проверок, чтобы прогон
// оставался годным как сторож для доказанного.
`define NOTE(msg, cond) \
    if (cond) $display("   ok: %0s", msg); \
    else      $display("   пока нет: %0s", msg);

`define CHECK(msg, cond) \
    if (cond) $display("   ok: %0s", msg); \
    else begin $display("   ПРОВАЛ: %0s", msg); errors = errors + 1; end

// ждём снятия занятости, но не дольше предела
task wait_ready(input integer limit_ns);
    integer t0; reg [7:0] s;
begin
    t0 = $time;
    s = 8'h01;
    while (s[0] && ($time - t0) < limit_ns) begin
        rd(2'b00, s);
        #100_000;
    end
    if (s[0]) begin
        $display("   ПРОВАЛ: занятость не снялась за %0d мс", limit_ns/1000000);
        errors = errors + 1;
    end
end
endtask

reg [7:0] st, trk;
integer busy_cycles;
integer got, bad;
reg [7:0] wr_data [0:255];
reg [7:0] trk_img [0:1023];
integer tp;
integer n_loop;
integer data_at;
reg [7:0] addr_fld [0:7];
integer n_intrq = 0;
reg intrq_d = 0;
integer n_rclk = 0, n_sync = 0, n_rawr = 0, n_idx = 0, n_a1 = 0, n_a2 = 0;
// ловушка записи: времена перепадов на линии записи, пока взведён WG
integer     n_wr = 0;
real        wr_t [0:20000];
// ---------------------------------------------- разбор того, что записано
// Перепады на линии записи превращаем обратно в ячейки, ищем синхромаркер и
// читаем байты. Это и есть доказательство: не «ядро что-то выдало», а «на
// носитель легло ровно то, что просили».
reg [0:40000] cells;
integer       n_cells;
reg [7:0]     dec_byte [0:400];
integer       n_dec, sync_at;
reg [15:0]    dec_crc;

task decode_written;
    integer i, k, gap, pos, b;
    reg [15:0] w;
begin
    // интервалы -> ячейки. В MFM между перепадами 2, 3 или 4 ячейки.
    n_cells = 0;
    cells[0] = 1'b1; n_cells = 1;
    for (i = 1; i < n_wr; i = i + 1) begin
        // Полячейки НЕ прибавляем: присваивание вещественного целому в Verilog
        // само округляет, и добавка давала двойное округление — 8125 нс
        // превращались в 5 ячеек вместо 4, то есть лишнюю ячейку на байт.
        gap = (wr_t[i] - wr_t[i-1]) / CELL_NS;
        if (gap < 1) gap = 1;
        for (k = 1; k < gap; k = k + 1) begin cells[n_cells] = 1'b0; n_cells = n_cells + 1; end
        cells[n_cells] = 1'b1; n_cells = n_cells + 1;
    end
    // ищем 4489
    sync_at = -1;
    for (i = 0; i + 16 <= n_cells && sync_at < 0; i = i + 1) begin
        w = 16'h0;
        for (k = 0; k < 16; k = k + 1) w = {w[14:0], cells[i+k]};
        if (w == 16'h4489) sync_at = i;
    end
    // с найденного места читаем байты: нечётные ячейки — данные
    n_dec = 0;
    if (sync_at >= 0) begin
        pos = sync_at;
        while (pos + 16 <= n_cells && n_dec < 400) begin
            b = 0;
            for (k = 0; k < 8; k = k + 1) b = (b << 1) | cells[pos + 2*k + 1];
            dec_byte[n_dec] = b[7:0]; n_dec = n_dec + 1;
            pos = pos + 16;
        end
    end
end
endtask



reg         wd_d = 1'b1;
integer     wg_up = 0;
integer     n_drq = 0;
reg         drq_d2 = 0;
reg         wg_d = 1'b0;
always @(posedge clk) begin
    wg_d <= dut.wg;
    if (dut.wg && !wg_d) begin wg_up = wg_up + 1; n_wr = 0; end
    intrq_d <= intrq;
    if (intrq && !intrq_d) n_intrq = n_intrq + 1;
    drq_d2 <= drq;
    if (drq && !drq_d2) n_drq = n_drq + 1;
    wd_d <= dut.FDC_WR_DATA;
    if (dut.wg && !dut.FDC_WR_DATA && wd_d && n_wr < 20000) begin
        wr_t[n_wr] = $realtime; n_wr = n_wr + 1;
    end
end
// покадровая сверка: биты, вдвинутые в линию задержки за окно трёх маркеров
reg         snap = 0;
reg         rclk_any = 0;
reg  [63:0] seen_bits = 0;
integer     n_bits = 0, snap_done = 0;
reg         win_d = 0;
always @(posedge clk) begin
    rclk_any <= dut.rclk;
    win_d    <= mark_win;
    if (mark_win && !win_d) begin seen_bits = 0; n_bits = 0; end
    if (mark_win && (dut.rclk !== rclk_any)) begin     // фронт RCLK, любой
        seen_bits = {seen_bits[62:0], dut.U16.rBIT};
        n_bits = n_bits + 1;
    end
    if (!mark_win && win_d && snap && !snap_done) begin
        snap_done = 1;
        $display("   СНИМОК окна трёх маркеров:");
        $display("     тактов RCLK за 48 ячеек: %0d (должно быть 48)", n_bits);
        $display("     принято : %012h", seen_bits[47:0]);
        $display("     ожидалось: 448944894489");
    end
end
reg rclk_d = 0, sync_d = 0, idx_d2 = 1;
always @(posedge clk) begin
    rclk_d <= dut.rclk; if (dut.rclk && !rclk_d) n_rclk = n_rclk + 1;
    sync_d <= dut.sync; if (dut.sync === 1'b1 && !sync_d) n_sync = n_sync + 1;
    if (dut.rawr === 1'b1) n_rawr = n_rawr + 1;
    // Сравнивать надо РОВНО то, что сравнивает детектор: у него в сравнении
    // участвует ещё не сдвинутый rBIT. Смотреть на один o3WORDS — это смотреть
    // на картину, сдвинутую на бит.
    if ({dut.U16.o3WORDS[14:0], dut.U16.rBIT} === 16'h4489) n_a1 = n_a1 + 1;
    if ({dut.U16.o3WORDS[46:0], dut.U16.rBIT} === 48'h448944894489) n_a2 = n_a2 + 1;
    idx_d2 <= index_n;  if (!index_n && idx_d2) n_idx = n_idx + 1;
end
real t0;

initial begin
    // Волны -- только по +vcd: без них прогон быстрее, а файл выходил на 788 МБ.
    if ($test$plusargs("vcd")) begin
        $dumpfile("tb_fdc1793.vcd"); $dumpvars(0, tb_fdc1793);
    end
    // Раньше здесь снаружи ставились начальные значения clk_16, w288, o3WORDS,
    // rBIT, rIPTRG и rIPTRG0: без них ядро в симуляции не стартовало. Теперь
    // они заданы в самом ядре (rtl/fdc, правки -- в README.md в корне), и
    // стенд обязан проходить без подпорок.
    #1000; nres = 1; #10000;

    $display("== 1. после сброса ВГ93 сам делает RESTORE");
    // Это не догадка про ядро, а буква даташита WD1793: по фронту /MR
    // микросхема выполняет Restore сама. Значит занятость сразу ВЗВЕДЕНА, и
    // головка уезжает на нулевую дорожку без единой команды от машины.
    rd(2'b00, st);
    $display("   состояние = %02h, автомат = %0d", st, dut.U14.rLAST_CURR_STATE);
        `CHECK("занятость взведена сама, без команды", st[0] === 1'b1);
    wait_ready(400_000_000);
    rd(2'b00, st);
    $display("   после самовосстановления: дорожка привода %0d, состояние %02h", track, st);
        `CHECK("самовосстановление увело головку на нулевую", track === 8'd0);
        `CHECK("разряд 2 = ДОР 0 после самовосстановления", st[2] === 1'b1);
    track = 8'd3;                      // вернём для следующей проверки

    $display("");
    $display("== 2. RESTORE с 3-й дорожки, темп 6 мс (0x08)");
    steps = 0;
    wr(2'b00, 8'h08);                  // Restore, h=0, V=0, r=00 -> 6 мс
    #100_000;
    rd(2'b00, st);
        `CHECK("занятость взвелась по команде", st[0] === 1'b1);
    wait_ready(200_000_000);
    rd(2'b00, st);
    rd(2'b01, trk);
    $display("   шагов сделано: %0d, дорожка привода %0d, регистр дорожки %02h, состояние %02h",
             steps, track, trk, st);
        `CHECK("головка доехала до нулевой дорожки", track === 8'd0);
        `CHECK("регистр дорожки обнулён", trk === 8'h00);
        `CHECK("разряд 2 состояния = ДОР 0", st[2] === 1'b1);
        `CHECK("разряд 4 = ошибка позиционирования снят", st[4] === 1'b0);

    $display("");
    $display("== 3. SEEK на дорожку 5 (0x18)");
    steps = 0;
    wr(2'b11, 8'd5);                   // регистр данных = куда ехать
    wr(2'b00, 8'h18);                  // Seek, 6 мс
    #100_000;
    wait_ready(200_000_000);
    rd(2'b01, trk);
    rd(2'b00, st);
    $display("   шагов %0d, дорожка привода %0d, регистр %0d, состояние %02h",
             steps, track, trk, st);
        `CHECK("сделано ровно 5 шагов", steps === 5);
        `CHECK("привод на 5-й дорожке", track === 8'd5);
        `CHECK("регистр дорожки = 5", trk === 8'd5);
        `CHECK("разряд 2 (ДОР 0) снят", st[2] === 1'b0);

    $display("");
    $display("== 4. ЗАМЕР: сколько ядро держит занятость, не трогая носитель");
    // Предупреждение из темы zx-pk про замену ВГ93: «некоторые программы могут
    // захотеть, чтобы контроллер обязательно побыл занят хотя бы один цикл
    // опроса». Если команда выполняется мгновенно, машина ни разу не застанет
    // занятость взведённой, и цикл ожидания сломается.
    //
    // Худший случай — Seek туда, где головка уже стоит, с V=0: шагов ноль,
    // проверки нет, носитель не нужен. Меряем по внутреннему rBUSY, а не через
    // шину: через шину мерка сама по себе занимает время.
    wr(2'b11, 8'd5);                   // цель = текущая дорожка
    busy_cycles = 0;
    fork
        begin : counter
            integer n;
            for (n = 0; n < 32000; n = n + 1) begin   // окно 1 мс при 32 МГц
                @(posedge clk);
                if (dut.U14.rBUSY) busy_cycles = busy_cycles + 1;
            end
        end
        wr(2'b00, 8'h18);              // Seek, V=0, темп 6 мс
    join
    $display("   занятость держалась %0d тактов = %0d нс",
             busy_cycles, busy_cycles * 2 * 15.625);
        `CHECK("занятость вообще взводится", busy_cycles > 0);
        // Это ХАРАКТЕРИСТИКА, а не приговор: число закрепляем, чтобы заметить,
        // если чужое ядро однажды сменится и поведение уедет.
        `CHECK("замер прежний (8 тактов clk_16)", busy_cycles == 16);
    if (busy_cycles * 2 * 15.625 < 5000) begin
        $display("   ВНИМАНИЕ: это короче одного опроса машины.");
        $display("   Автомат ядра крутится на 16 МГц, а у живой ВГ93 на 1 МГц —");
        $display("   счётчики задержек пересчитаны, длительность состояний нет.");
        $display("   Обёртка ОБЯЗАНА продлевать занятость, иначе цикл ожидания");
        $display("   в программе не застанет её ни разу. Разбор — README.md в корне.");
    end

    $display("");
    $display("== 5. ТИП II: Read Sector (0x80) с настоящего потока MFM");
    // Головка на 5-й дорожке после проверки 3, на «дискете» лежит дорожка 5,
    // сторона 0, сектор 1, 256 байт со значениями 0..255.
    got = 0; bad = 0;
    // разведка: доходит ли поток до ядра вообще
    $display("   ФАПЧ: rclk=%b rawr=%b vfoe=%b, синхро=%b, старт=%b",
             dut.rclk, dut.rawr, dut.vfoe, dut.sync, dut.start);
    wr(2'b10, 8'd1);                   // регистр сектора
    snap = 1;
    wr(2'b00, 8'h80);                  // Read Sector, одиночный, E=0
    #500_000;
    $display("   через 0,5 мс: vfoe=%b hld=%b hrdy2=%b head_in_pos=%b wg=%b busy=%b",
             dut.vfoe, dut.U14.rHLD, dut.U14.rHRDY2, dut.U14.rHEAD_IN_POS,
             dut.wg, dut.U14.rBUSY);
    #500_000;
    $display("   через 0,5 мс: vfoe=%b hld=%b hrdy2=%b head_in_pos=%b wg=%b busy=%b",
             dut.vfoe, dut.U14.rHLD, dut.U14.rHRDY2, dut.U14.rHEAD_IN_POS,
             dut.wg, dut.U14.rBUSY);
    t0 = $time;
    while (got < 256 && ($time - t0) < 500_000_000) begin
        @(posedge clk);
        if (drq) begin
            rd(2'b11, st);
            if (st !== sector_data[got]) begin
                if (bad < 5) $display("   байт %0d: получено %02h, ожидалось %02h",
                                      got, st, sector_data[got]);
                bad = bad + 1;
            end
            got = got + 1;
        end
    end
    #2_000_000;
    $display("   одиночных A1 (как видит детектор): %0d, ПОЛНЫХ троек: %0d", n_a1, n_a2);
    $display("   счётчики: тактов ФАПЧ %0d, срабатываний детектора %0d, импульсов с головки %0d, индексов %0d",
             n_rclk, n_sync, n_rawr, n_idx);
    $display("   после команды: sync=%b start=%b byte_2_read=%b byte_2_main=%02h ip_cnt=%0d",
             dut.sync, dut.start, dut.byte_2_read, dut.byte_2_main, dut.ip_cnt);
    rd(2'b00, st);
    $display("   получено байтов: %0d, из них неверных: %0d, состояние %02h", got, bad, st);
        `CHECK("сектор найден и прочитан целиком", got === 256);
        `CHECK("все 256 байт совпали", bad === 0);
        `CHECK("разряд 3 (ошибка CRC) снят", st[3] === 1'b0);
        `CHECK("разряд 4 (сектор не найден) снят", st[4] === 1'b0);
        `CHECK("разряд 2 (потеря данных) снят", st[2] === 1'b0);

    $display("");
    $display("== 6. ТИП II: Write Sector (0xA0)");
    // Ядро само найдёт поле адреса того же сектора, дождётся места под поле
    // данных и взведёт WG. Наше дело — подавать байты по ЗПД и не отстать:
    // отстанем — ядро выставит разряд 2, «потеря данных».
    for (gi = 0; gi < 256; gi = gi + 1) wr_data[gi] = 8'hC0 + gi[7:0];
    got = 0; bad = 0; wg_up = 0;
    wr(2'b10, 8'd1);                   // сектор
    wr(2'b00, 8'hA0);                  // Write Sector, одиночный
    t0 = $time;
    while (got < 256 && ($time - t0) < 500_000_000) begin
        @(posedge clk);
        if (drq) begin wr(2'b11, wr_data[got]); got = got + 1; end
    end
    #5_000_000;
    rd(2'b00, st);
    $display("   отдано байтов: %0d, WG взводился %0d раз, перепадов записано %0d, состояние %02h",
             got, wg_up, n_wr, st);
        `CHECK("ядро приняло все 256 байт", got === 256);
        `CHECK("разрешение записи взводилось", wg_up > 0);
        `CHECK("на линию записи пошли перепады", n_wr > 500);
        `CHECK("разряд 2 (потеря данных) снят", st[2] === 1'b0);
        `CHECK("разряд 4 (сектор не найден) снят", st[4] === 1'b0);
        `CHECK("разряд 6 (защита записи) снят", st[6] === 1'b0);
        `CHECK("разряд 5 (сбой записи) снят", st[5] === 1'b0);

    decode_written;
    begin : dumpcells
        integer q, r; reg [15:0] w;
        $write("   ячейки от маркера:");
        for (q = 0; q < 6; q = q + 1) begin
            w = 16'h0;
            for (r = 0; r < 16; r = r + 1) w = {w[14:0], cells[sync_at + q*16 + r]};
            $write(" %04h", w);
        end
        $display("");
        $display("   ожидалось: 4489 4489 4489 <FB>");
    end
    $display("   разобрано: ячеек %0d, синхромаркер на позиции %0d, байтов %0d",
             n_cells, sync_at, n_dec);
    if (n_dec >= 6)
        $display("   первые байты: %02h %02h %02h %02h %02h %02h",
                 dec_byte[0], dec_byte[1], dec_byte[2], dec_byte[3], dec_byte[4], dec_byte[5]);
        `CHECK("синхромаркер A1 найден в записанном", sync_at >= 0);
        `CHECK("записаны три A1 подряд",
               n_dec >= 3 && dec_byte[0] === 8'hA1 && dec_byte[1] === 8'hA1 && dec_byte[2] === 8'hA1);
        `CHECK("за ними метка поля данных FB", n_dec >= 4 && dec_byte[3] === 8'hFB);
    bad = 0;
    for (gi = 0; gi < 256; gi = gi + 1)
        if (n_dec >= 4 + 256 && dec_byte[4 + gi] !== wr_data[gi]) bad = bad + 1;
    $display("   байтов данных не совпало: %0d", bad);
        `CHECK("все 256 байт данных легли верно", bad === 0 && n_dec >= 260);
    // CRC по A1,A1,A1,FB и данным
    dec_crc = 16'hFFFF;
    for (gi = 0; gi < 4 + 256; gi = gi + 1) begin : crcloop
        integer q;
        dec_crc = dec_crc ^ {dec_byte[gi], 8'h00};
        for (q = 0; q < 8; q = q + 1)
            dec_crc = dec_crc[15] ? {dec_crc[14:0],1'b0} ^ 16'h1021 : {dec_crc[14:0],1'b0};
    end
    if (n_dec >= 262)
        $display("   CRC: записана %02h%02h, посчитана %04h",
                 dec_byte[260], dec_byte[261], dec_crc);
        `CHECK("CRC записана верно",
               n_dec >= 262 && {dec_byte[260], dec_byte[261]} === dec_crc);

    $display("");
    $display("== 7. ТИП III: Write Track (0xF0) — форматирование");
    // Самая капризная команда, и единственная, которой «Вектор-06Ц» форматирует
    // (format.com МикроДОС выдаёт F0 и F4). Служебные байты подставляет САМО ЯДРО:
    // F5 -> A1 с выбитым тактовым разрядом и заводом CRC, F6 -> C2, F7 -> два
    // байта CRC. Проверяем именно подстановку, а не то, что команда прошла.
    tp = 0;
    for (gi = 0; gi < 32; gi = gi + 1) trk_img[tp+gi] = 8'h4E;   tp = tp + 32;
    for (gi = 0; gi < 12; gi = gi + 1) trk_img[tp+gi] = 8'h00;   tp = tp + 12;
    trk_img[tp] = 8'hF5; trk_img[tp+1] = 8'hF5; trk_img[tp+2] = 8'hF5; tp = tp + 3;
    trk_img[tp] = 8'hFE;                                        tp = tp + 1;
    trk_img[tp] = 8'd7;  trk_img[tp+1] = 8'd0;
    trk_img[tp+2] = 8'd3; trk_img[tp+3] = 8'h01;                tp = tp + 4;
    trk_img[tp] = 8'hF7;                                        tp = tp + 1;
    for (gi = 0; gi < 22; gi = gi + 1) trk_img[tp+gi] = 8'h4E;   tp = tp + 22;
    for (gi = 0; gi < 12; gi = gi + 1) trk_img[tp+gi] = 8'h00;   tp = tp + 12;
    trk_img[tp] = 8'hF5; trk_img[tp+1] = 8'hF5; trk_img[tp+2] = 8'hF5; tp = tp + 3;
    trk_img[tp] = 8'hFB;                                        tp = tp + 1;
    data_at = tp;                      // где данные лежат В ОБРАЗЕ
    // Узор НЕ должен задевать F5, F6 и F7: внутри Write Track это служебные
    // байты, и ядро подставит вместо них маркеры и CRC. Первый заход на 0x50+i
    // попал на F5 ровно на 165-м байте, и расшифровка честно показала A1.
    for (gi = 0; gi < 256; gi = gi + 1)
        trk_img[tp+gi] = ((8'h50 + gi[7:0]) >= 8'hF5 && (8'h50 + gi[7:0]) <= 8'hF7)
                         ? 8'h4E : (8'h50 + gi[7:0]);
    tp = tp + 256;
    trk_img[tp] = 8'hF7;                                        tp = tp + 1;
    for (gi = 0; gi < 24; gi = gi + 1) trk_img[tp+gi] = 8'h4E;   tp = tp + 24;

    got = 0; wg_up = 0; n_wr = 0; n_drq = 0;
    wr(2'b00, 8'hF0);                  // Write Track
    // Цикл подачи: гоним байты, пока ядро занято. Выход по снятию занятости,
    // а не по времени: Write Track идёт от индекса до индекса, и сколько это
    // тактов — дело носителя, а не наше.
    n_loop = 0;
    while (dut.U14.rBUSY && n_loop < 2000000) begin
        @(posedge clk);
        n_loop = n_loop + 1;
        if (drq) begin
            wr(2'b11, (got < tp) ? trk_img[got] : 8'h4E);
            got = got + 1;
        end
    end
    #2_000_000;
    rd(2'b00, st);
    $display("   отдано байтов %0d (образ %0d), WG взводился %0d раз, перепадов %0d, состояние %02h",
             got, tp, wg_up, n_wr, st);
    $display("   подъёмов ЗПД за команду: %0d, стадия %0d, r_drq_r_dreg=%b, oWG=%b",
             n_drq, dut.U14.rSTAGE, dut.r_drq_r_dreg, dut.U14.oWG);
        `CHECK("разрешение записи взводилось ровно один раз", wg_up === 1);
        // Write Track идёт ОТ ИНДЕКСА ДО ИНДЕКСА, то есть ровно оборот. Сколько
        // байтов туда влезет — дело носителя, а не образа: лишнее ядро просто не
        // возьмёт. Проверяем, что взяло весь образ и остановилось само.
        `CHECK("ядро забрало весь образ", got >= tp);
        `CHECK("ядро остановилось само, по индексу", dut.U14.rBUSY === 1'b0);
        `CHECK("разряд 2 (потеря данных) снят", st[2] === 1'b0);

    decode_written;
    $display("   разобрано байтов %0d, первые: %02h %02h %02h %02h %02h %02h %02h %02h %02h %02h",
             n_dec, dec_byte[0], dec_byte[1], dec_byte[2], dec_byte[3], dec_byte[4],
             dec_byte[5], dec_byte[6], dec_byte[7], dec_byte[8], dec_byte[9]);
        `CHECK("F5 F5 F5 превратились в A1 A1 A1",
               n_dec > 10 && dec_byte[0] === 8'hA1 && dec_byte[1] === 8'hA1 && dec_byte[2] === 8'hA1);
        `CHECK("метка адреса FE на месте", n_dec > 10 && dec_byte[3] === 8'hFE);
        `CHECK("номера дорожки, стороны, сектора и длины легли верно",
               n_dec > 10 && dec_byte[4] === 8'd7 && dec_byte[5] === 8'd0
                          && dec_byte[6] === 8'd3 && dec_byte[7] === 8'h01);
    dec_crc = 16'hFFFF;
    for (gi = 0; gi < 8; gi = gi + 1) begin : crcid
        integer q;
        dec_crc = dec_crc ^ {dec_byte[gi], 8'h00};
        for (q = 0; q < 8; q = q + 1)
            dec_crc = dec_crc[15] ? {dec_crc[14:0],1'b0} ^ 16'h1021 : {dec_crc[14:0],1'b0};
    end
    $display("   CRC поля адреса: записана %02h%02h, посчитана %04h",
             dec_byte[8], dec_byte[9], dec_crc);
        `CHECK("F7 превратился в верную CRC поля адреса",
               n_dec > 10 && {dec_byte[8], dec_byte[9]} === dec_crc);

    // поле данных: после поля адреса идёт промежуток 22 x 4E и 12 нулей
    $display("   поле данных с байта 44: %02h %02h %02h %02h %02h %02h",
             dec_byte[44], dec_byte[45], dec_byte[46], dec_byte[47], dec_byte[48], dec_byte[49]);
        `CHECK("второй F5 F5 F5 тоже дал A1 A1 A1",
               n_dec > 50 && dec_byte[44] === 8'hA1 && dec_byte[45] === 8'hA1 && dec_byte[46] === 8'hA1);
        `CHECK("метка поля данных FB на месте", n_dec > 50 && dec_byte[47] === 8'hFB);
    bad = 0;
    for (gi = 0; gi < 256; gi = gi + 1)
        if (n_dec > 48 + 256 && dec_byte[48 + gi] !== trk_img[data_at + gi]) bad = bad + 1;
    begin : firstbad
        integer q; integer fb;
        fb = -1;
        for (q = 0; q < 256; q = q + 1)
            if (fb < 0 && n_dec > 48+q && dec_byte[48+q] !== trk_img[data_at+q]) fb = q;
        if (fb >= 0)
            $display("   первое расхождение на байте %0d: %02h вместо %02h; соседи %02h %02h %02h",
                     fb, dec_byte[48+fb], trk_img[data_at+fb],
                     dec_byte[47+fb], dec_byte[49+fb], dec_byte[50+fb]);
    end
        `CHECK("все 256 байт поля данных легли верно", bad === 0 && n_dec > 305);
    dec_crc = 16'hFFFF;
    for (gi = 44; gi < 48 + 256; gi = gi + 1) begin : crcdt
        integer q;
        dec_crc = dec_crc ^ {dec_byte[gi], 8'h00};
        for (q = 0; q < 8; q = q + 1)
            dec_crc = dec_crc[15] ? {dec_crc[14:0],1'b0} ^ 16'h1021 : {dec_crc[14:0],1'b0};
    end
    $display("   CRC поля данных: записана %02h%02h, посчитана %04h",
             dec_byte[304], dec_byte[305], dec_crc);
        `CHECK("второй F7 дал верную CRC поля данных",
               n_dec > 305 && {dec_byte[304], dec_byte[305]} === dec_crc);

    $display("");
    $display("== 8. ТИП IV: Force Interrupt (0xD8) — прервать идущую команду");
    // Единственная команда, которую машина может выдать поверх занятости.
    // format.com шлёт именно D8 — немедленное прерывание.
    track = 8'd40;
    wr(2'b00, 8'h03);                  // Restore, темп 30 мс: поедет долго
    #1_000_000;
    rd(2'b00, st);
        `CHECK("длинная команда пошла", st[0] === 1'b1);
    steps = 0;
    #40_000_000;                       // дать проехать часть пути
    $display("   через 40 мс: шагов %0d, дорожка %0d, занятость %b", steps, track, dut.U14.rBUSY);
        `CHECK("к этому времени команда ещё идёт", dut.U14.rBUSY === 1'b1);
    n_intrq = 0;
    wr(2'b00, 8'hD8);                  // Force Interrupt, немедленное
    #200_000;
    rd(2'b00, st);
    $display("   после D8: занятость %b, состояние %02h, подъёмов ПРР %0d, шагов стало %0d",
             dut.U14.rBUSY, st, n_intrq, steps);
        `CHECK("занятость снята прерыванием", st[0] === 1'b0);
        `CHECK("ПРР выставлено", n_intrq > 0);
    gi = steps;
    #20_000_000;
        `CHECK("шаги прекратились", steps === gi);

    $display("");
    $display("== 9. ТИП III: Read Address (0xC0)");
    // МикроДОС её не выдаёт, но начальный загрузчик может: выборка по .COM
    // системной дискеты загрузчика не охватывала.
    // Прерывание бросило головку на 38-й дорожке, обратно это 33 шага по 6 мс.
    wr(2'b11, 8'd5); wr(2'b00, 8'h18); wait_ready(400_000_000);
    got = 0; bad = 0;
    wr(2'b00, 8'hC0);                  // Read Address
    t0 = $time;
    while (got < 6 && ($time - t0) < 200_000_000) begin
        @(posedge clk);
        if (drq) begin rd(2'b11, st); addr_fld[got] = st; got = got + 1; end
    end
    #2_000_000;
    rd(2'b00, st);
    $display("   прочитано %0d байт: %02h %02h %02h %02h %02h %02h, состояние %02h",
             got, addr_fld[0], addr_fld[1], addr_fld[2], addr_fld[3],
             addr_fld[4], addr_fld[5], st);
        `CHECK("поле адреса выдано целиком, шесть байт", got === 6);
        `CHECK("дорожка из поля адреса совпала с носителем", addr_fld[0] === TRK_ON_DISK);
        `CHECK("сторона совпала", addr_fld[1] === SIDE_ON_DISK);
        `CHECK("номер сектора совпал", addr_fld[2] === SEC_ON_DISK);
        `CHECK("длина сектора 01 = 256 байт", addr_fld[3] === 8'h01);
        `CHECK("разряд 3 (ошибка CRC) снят", st[3] === 1'b0);
    // CRC поля адреса считается по A1 A1 A1 FE и четырём номерам
    dec_crc = 16'hFFFF;
    for (gi = 0; gi < 8; gi = gi + 1) begin : crcra
        integer q; reg [7:0] bb;
        bb = (gi < 3) ? 8'hA1 : (gi == 3) ? 8'hFE : addr_fld[gi-4];
        dec_crc = dec_crc ^ {bb, 8'h00};
        for (q = 0; q < 8; q = q + 1)
            dec_crc = dec_crc[15] ? {dec_crc[14:0],1'b0} ^ 16'h1021 : {dec_crc[14:0],1'b0};
    end
    $display("   CRC поля адреса: выдана %02h%02h, посчитана %04h",
             addr_fld[4], addr_fld[5], dec_crc);
        `CHECK("CRC выдана та, что лежит на носителе",
               {addr_fld[4], addr_fld[5]} === dec_crc);

    $display("");
    $display("== 10. ТИП III: Read Track (0xE0)");
    // По паспорту WD1793: чтение от индекса до следующего индекса, байт по DRQ
    // на каждый, кадр байтов подстраивается по меткам. Поле адреса с CRC, метка
    // данных, данные и их CRC должны прийти верными; байты промежутков до первой
    // метки -- как получится (сдвиг кадра). Её выдают копировщики и «доктора»
    // дискет (DOCTOR, REANIMAT на системной дискете T-34).
    got = 0;
    wr(2'b00, 8'hE0);                  // Read Track, без задержки 15 мс
    t0 = $time;
    while ((got == 0 || dut.U14.rBUSY) && ($time - t0) < 100_000_000) begin
        @(posedge clk);
        if (drq && got < 1024) begin rd(2'b11, st); trk_img[got] = st; got = got + 1; end
    end
    #2_000_000;
    rd(2'b00, st);
    $display("   прочитано %0d байт за %0d мс, состояние %02h", got, ($time - t0) / 1_000_000, st);
        `CHECK("команда кончилась сама", dut.U14.rBUSY === 1'b0);
        `CHECK("разряд 2 (потеря данных) снят", st[2] === 1'b0);
    begin : rt_check
        integer i, j, k, id_at, dm_at, n_id, bad_d;
        reg [15:0] c;
        id_at = -1; dm_at = -1; n_id = 0;
        for (i = 0; i + 3 < got; i = i + 1)
            if (trk_img[i] === 8'hA1 && trk_img[i+1] === 8'hA1 &&
                trk_img[i+2] === 8'hA1 && trk_img[i+3] === 8'hFE) begin
                n_id = n_id + 1;
                if (id_at < 0) id_at = i;
            end
        if (id_at >= 0)
            for (i = id_at + 4; i + 3 < got && dm_at < 0; i = i + 1)
                if (trk_img[i] === 8'hA1 && trk_img[i+1] === 8'hA1 &&
                    trk_img[i+2] === 8'hA1 && trk_img[i+3] === 8'hFB) dm_at = i;
        $display("   A1 A1 A1 FE на %0d, A1 A1 A1 FB на %0d, байт до первой метки %0d",
                 id_at, dm_at, id_at);
        $write("   до метки:");
        for (i = 0; i < id_at && i < 48; i = i + 1) $write(" %02h", trk_img[i]);
        $display("");
        `CHECK("поле адреса A1 A1 A1 FE найдено", id_at >= 0);
        `CHECK("ровно одно: прочитан ровно один оборот", n_id === 1);
        // Промежутки отдаются, как у WD1793 (паспорт FD179X-01, октябрь 1979,
        // с. 13: «Gaps are included in the input data stream», кадр байтов
        // подстраивается по каждой метке): до первой метки кадр произвольный
        // (16 x 4E приходят сдвинутыми), на метке встаёт на место. До 02.10.2026
        // декодер молчал до первой метки и 28 байт пропадали -- починено
        // признаком oRDTRK из Main_CTRL (наше), см. fdc_core.v.
        `CHECK("промежуток до первой метки выдан: 16 x 4E и 12 x 00, не меньше 28 байт", id_at >= 28);
        // Оборот модели: индекс 500 мкс + 374 байта по 32 мкс = 12,468 мс, то
        // есть 389,6 байта.
        `CHECK("за оборот 387..392 байта -- весь оборот, а не с первой метки", got >= 387 && got <= 392);
        if (id_at >= 0) begin
            `CHECK("дорожка, сторона, сектор, длина -- как на носителе",
                   trk_img[id_at+4] === TRK_ON_DISK && trk_img[id_at+5] === SIDE_ON_DISK &&
                   trk_img[id_at+6] === SEC_ON_DISK && trk_img[id_at+7] === 8'h01);
            c = 16'hFFFF;
            for (j = id_at; j < id_at + 8; j = j + 1) begin
                c = c ^ {trk_img[j], 8'h00};
                for (k = 0; k < 8; k = k + 1) c = c[15] ? {c[14:0],1'b0} ^ 16'h1021 : {c[14:0],1'b0};
            end
            `CHECK("CRC поля адреса верная", {trk_img[id_at+8], trk_img[id_at+9]} === c);
        end
        `CHECK("метка данных A1 A1 A1 FB найдена", dm_at >= 0);
        if (dm_at >= 0) begin
            bad_d = 0;
            for (j = 0; j < 256; j = j + 1)
                if (dm_at + 4 + j >= got || trk_img[dm_at+4+j] !== sector_data[j]) bad_d = bad_d + 1;
            `CHECK("все 256 байт данных верны", bad_d === 0);
            c = 16'hFFFF;
            for (j = dm_at; j < dm_at + 4 + 256; j = j + 1) begin
                c = c ^ {trk_img[j], 8'h00};
                for (k = 0; k < 8; k = k + 1) c = c[15] ? {c[14:0],1'b0} ^ 16'h1021 : {c[14:0],1'b0};
            end
            `CHECK("CRC поля данных верная",
                   dm_at + 261 < got && {trk_img[dm_at+260], trk_img[dm_at+261]} === c);
            `CHECK("после данных прочитан промежуток 4E до индекса",
                   dm_at + 262 < got && trk_img[dm_at+262] === 8'h4E);
        end
    end

    $display("");
    if (errors == 0) $display("ВСЕ ПРОВЕРКИ ПРОЙДЕНЫ");
    else             $display("ПРОВАЛОВ: %0d", errors);
    $finish;
end

endmodule

`default_nettype wire
