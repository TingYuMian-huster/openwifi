## 测试平台设计思路

```verilog
`timescale 1 ns / 1 ps

module tb_tx_system();

// 参数定义
parameter integer C_S00_AXIS_TDATA_WIDTH = 64;
parameter integer WIFI_TX_BRAM_ADDR_WIDTH = 10;
parameter integer WIFI_TX_BRAM_DATA_WIDTH = 64;
parameter integer WIFI_TX_BRAM_WEN_WIDTH = 8;
parameter integer MAX_NUM_DMA_SYMBOL = 8192;
parameter integer MAX_BIT_NUM_DMA_SYMBOL = 14;

// 时钟和复位
reg clk;
reg rstn;
reg s00_axis_aclk;
reg s00_axis_aresetn;

// AXI Stream 接口信号
reg [C_S00_AXIS_TDATA_WIDTH-1:0] s00_axis_tdata;
reg [(C_S00_AXIS_TDATA_WIDTH/8)-1:0] s00_axis_tstrb;
reg s00_axis_tlast;
reg s00_axis_tvalid;
wire s00_axis_tready;

// 寄存器信号（模拟PS配置）
reg [31:0] slv_reg0;
reg [31:0] slv_reg4;
reg [31:0] slv_reg8;
reg [31:0] slv_reg10;
reg [31:0] slv_reg15;
reg [31:0] slv_reg17;

// 模块间连接信号
wire [2:0] tx_queue_idx;
wire s_axis_recv_data_from_high;
wire [C_S00_AXIS_TDATA_WIDTH-1:0] s_axis_data_to_acc;
wire s_axis_emptyn_to_acc;
wire tx_bit_intf_acc_ask_data_from_s_axis;

// tx_bit_intf 输出信号
wire [WIFI_TX_BRAM_DATA_WIDTH-1:0] data_to_acc;
wire [WIFI_TX_BRAM_ADDR_WIDTH-1:0] bram_addr;
wire tx_end_from_acc;

// 其他控制信号
wire phy_tx_auto_start_mode;
wire [9:0] phy_tx_auto_start_num_dma_symbol_th;
wire [6:0] tx_config_fifo_data_count0, tx_config_fifo_data_count1, tx_config_fifo_data_count2, tx_config_fifo_data_count3;
wire tx_iq_fifo_empty;
wire [13:0] send_cts_toself_wait_sifs_top;
wire [47:0] mac_addr;
wire tx_try_complete;
wire retrans_in_progress;
wire start_retrans;
wire start_tx_ack;
wire [3:0] slice_en;
wire backoff_done;
wire tx_bb_is_ongoing;
wire ack_tx_flag;
wire wea_from_xpu;
wire [9:0] addra_from_xpu;
wire [C_S00_AXIS_TDATA_WIDTH-1:0] dina_from_xpu;
wire tx_pkt_need_ack;
wire [3:0] tx_pkt_retrans_limit;
wire use_ht_aggr;
wire quit_retrans;
wire reset_backoff;
wire high_trigger;
wire [5:0] bd_wr_idx;
wire [WIFI_TX_BRAM_DATA_WIDTH-1:0] douta;
wire cts_toself_bb_is_ongoing;
wire cts_toself_rf_is_ongoing;
wire phy_tx_start;
wire fcs_in_strobe;
wire tx_control_state_idle;
wire tsf_pulse_1M;

// FIFO数据计数
wire [MAX_BIT_NUM_DMA_SYMBOL-1:0] s_axis_fifo_data_count0, s_axis_fifo_data_count1, s_axis_fifo_data_count2, s_axis_fifo_data_count3;

// 其他信号
wire [2:0] linux_prio;
wire [5:0] pkt_cnt;

// 时钟生成
always #5 clk = ~clk;
always #5 s00_axis_aclk = ~s00_axis_aclk; // 100MHz

// 模块实例化
tx_intf_s_axis #(
    .C_S_AXIS_TDATA_WIDTH(C_S00_AXIS_TDATA_WIDTH),
    .MAX_NUM_DMA_SYMBOL(MAX_NUM_DMA_SYMBOL),
    .MAX_BIT_NUM_DMA_SYMBOL(MAX_BIT_NUM_DMA_SYMBOL)
) tx_intf_s_axis_i (
    .S_AXIS_ACLK(s00_axis_aclk),
    .S_AXIS_ARESETN(s00_axis_aresetn & (~slv_reg0[2])),
    .S_AXIS_TREADY(s00_axis_tready),
    .S_AXIS_TDATA(s00_axis_tdata),
    .S_AXIS_TSTRB(s00_axis_tstrb),
    .S_AXIS_TLAST(s00_axis_tlast),
    .S_AXIS_TVALID(s00_axis_tvalid),
    .S_AXIS_NUM_DMA_SYMBOL_raw(slv_reg8[12:0]),
    
    .s_axis_recv_data_from_high(s_axis_recv_data_from_high),
    
    .tx_queue_idx_indication_from_ps(slv_reg8[19:18]),
    .tx_queue_idx(tx_queue_idx),
    .endless_mode(0),
    .data_count0(s_axis_fifo_data_count0),
    .data_count1(s_axis_fifo_data_count1),
    .data_count2(s_axis_fifo_data_count2),
    .data_count3(s_axis_fifo_data_count3),
    .DATA_TO_ACC(s_axis_data_to_acc),
    .EMPTYN_TO_ACC(s_axis_emptyn_to_acc),
    .ACC_ASK_DATA(tx_bit_intf_acc_ask_data_from_s_axis & (~slv_reg10[0]))
);

tx_bit_intf #(
    .C_S00_AXIS_TDATA_WIDTH(C_S00_AXIS_TDATA_WIDTH),
    .WIFI_TX_BRAM_ADDR_WIDTH(WIFI_TX_BRAM_ADDR_WIDTH),
    .WIFI_TX_BRAM_DATA_WIDTH(WIFI_TX_BRAM_DATA_WIDTH),
    .WIFI_TX_BRAM_WEN_WIDTH(WIFI_TX_BRAM_WEN_WIDTH)
) tx_bit_intf_i (
    .rstn(s00_axis_aresetn & (~slv_reg0[6])),
    .clk(s00_axis_aclk),

    .fcs_in_strobe(fcs_in_strobe),
    
    // recv bits from s_axis
    .tx_queue_idx(tx_queue_idx),
    .linux_prio(linux_prio),
    .pkt_cnt(pkt_cnt),
    .data_from_s_axis(s_axis_data_to_acc),
    .ask_data_from_s_axis(tx_bit_intf_acc_ask_data_from_s_axis),
    .emptyn_from_s_axis(s_axis_emptyn_to_acc),
    
    // src indication
    .auto_start_mode(phy_tx_auto_start_mode),
    .num_dma_symbol_th(phy_tx_auto_start_num_dma_symbol_th),
    .tx_config(slv_reg8),
    .tx_queue_idx_indication_from_ps(slv_reg8[19:18]),
    .phy_hdr_config(slv_reg17),
    .ampdu_action_config(slv_reg15),
    .s_axis_recv_data_from_high(s_axis_recv_data_from_high),
    .start(phy_tx_start),

    .tx_config_fifo_data_count0(tx_config_fifo_data_count0), 
    .tx_config_fifo_data_count1(tx_config_fifo_data_count1),
    .tx_config_fifo_data_count2(tx_config_fifo_data_count2), 
    .tx_config_fifo_data_count3(tx_config_fifo_data_count3),

    .tx_iq_fifo_empty(tx_iq_fifo_empty),
    .cts_toself_config(slv_reg4),
    .send_cts_toself_wait_sifs_top(send_cts_toself_wait_sifs_top),
    .mac_addr(mac_addr),
    .tx_try_complete(tx_try_complete),
    .retrans_in_progress(retrans_in_progress),
    .start_retrans(start_retrans),
    .start_tx_ack(start_tx_ack),
    .slice_en(slice_en),
    .backoff_done(backoff_done),
    .tx_bb_is_ongoing(tx_bb_is_ongoing),
    .ack_tx_flag(ack_tx_flag),
    .wea_from_xpu(wea_from_xpu),
    .addra_from_xpu(addra_from_xpu),
    .dina_from_xpu(dina_from_xpu),
    .tx_pkt_need_ack(tx_pkt_need_ack),
    .tx_pkt_retrans_limit(tx_pkt_retrans_limit),
    .use_ht_aggr(use_ht_aggr),
    .quit_retrans(quit_retrans),
    .reset_backoff(reset_backoff),
    .high_trigger(high_trigger),
    .tx_control_state_idle(tx_control_state_idle),
    .bd_wr_idx(bd_wr_idx),
    .douta(douta),
    .cts_toself_bb_is_ongoing(cts_toself_bb_is_ongoing),
    .cts_toself_rf_is_ongoing(cts_toself_rf_is_ongoing),
      
    // to send out to wifi tx module
    .tx_end_from_acc(tx_end_from_acc),
    .bram_data_to_acc(data_to_acc),
    .bram_addr(bram_addr),

    .tsf_pulse_1M(tsf_pulse_1M)
);

// 测试控制变量
integer i;
reg [31:0] test_data [0:15]; // 测试数据存储

// 初始化
initial begin
    // 初始化时钟和复位
    clk = 0;
    s00_axis_aclk = 0;
    rstn = 0;
    s00_axis_aresetn = 0;
    
    // 初始化寄存器
    slv_reg0 = 32'h0;        // 控制寄存器
    slv_reg4 = 32'h0;        // CTS配置
    slv_reg8 = 32'h0;        // TX配置
    slv_reg10 = 32'h0;       // 额外控制
    slv_reg15 = 32'h0;       // A-MPDU配置
    slv_reg17 = 32'h0;       // PHY头配置
    
    // 初始化AXI信号
    s00_axis_tdata = 0;
    s00_axis_tstrb = 8'hFF;
    s00_axis_tlast = 0;
    s00_axis_tvalid = 0;
    
    // 初始化测试数据
    for (i = 0; i < 16; i = i + 1) begin
        test_data[i] = i + 1;
    end
    
    // 释放复位
    #100;
    rstn = 1;
    s00_axis_aresetn = 1;
    
    #50;
    
    // 配置系统参数
    configure_system();
    
    // 发送测试数据
    send_test_data();
    
    // 运行一段时间后结束
    #10000;
    $display("Simulation completed successfully");
    $finish;
end

// 系统配置任务
task configure_system;
begin
    // 配置TX参数
    slv_reg8 = 32'h0000_020A; // 设置DMA符号数=10，队列0
    
    // 配置PHY头：长度=64字节，非HT模式，速率6Mbps
    slv_reg17 = 32'h0000_0040; 
    
    // 配置A-MPDU参数
    slv_reg15 = 32'h0040_0800; // buf_size=8, max_tx_bytes=1024
    
    // 配置CTS参数
    slv_reg4 = 32'h0000_0064; // CTS持续时间=100us
    
    #100;
end
endtask

// 发送测试数据任务
task send_test_data;
integer j;
begin
    $display("Starting to send test data...");
    
    // 等待系统就绪
    wait(s00_axis_tready == 1);
    #20;
    
    // 发送8个64位数据包
    for (j = 0; j < 8; j = j + 1) begin
        @(posedge s00_axis_aclk);
        s00_axis_tvalid = 1;
        s00_axis_tdata = {test_data[2*j+1], test_data[2*j]};
        
        if (j == 7) begin
            s00_axis_tlast = 1; // 最后一个包
        end
        
        // 等待TREADY
        do begin
            @(posedge s00_axis_aclk);
        end while (s00_axis_tready == 0);
    end
    
    // 结束传输
    @(posedge s00_axis_aclk);
    s00_axis_tvalid = 0;
    s00_axis_tlast = 0;
    
    $display("Test data transmission completed");
end
endtask

// 模拟其他模块的输入信号
assign phy_tx_auto_start_mode = 1'b1;
assign phy_tx_auto_start_num_dma_symbol_th = 10'd8;
assign tx_iq_fifo_empty = 1'b1; // 假设TX IQ FIFO为空
assign send_cts_toself_wait_sifs_top = 14'd16; // SIFS等待时间
assign mac_addr = 48'h001122334455;
assign tx_try_complete = 1'b0;
assign retrans_in_progress = 1'b0;
assign start_retrans = 1'b0;
assign start_tx_ack = 1'b0;
assign slice_en = 4'b0001; // 使能队列0
assign backoff_done = 1'b1; // 退避完成
assign tx_bb_is_ongoing = 1'b0; // TX基带未进行
assign ack_tx_flag = 1'b0; // 非ACK传输
assign wea_from_xpu = 1'b0; // 未从XPU写入
assign addra_from_xpu = 10'b0;
assign dina_from_xpu = 64'b0;
assign tx_control_state_idle = 1'b1; // TX控制状态机空闲
assign tsf_pulse_1M = 1'b0; // 1MHz TSF脉冲
assign fcs_in_strobe = 1'b0; // FCS strobe

// 模拟TX结束信号
assign tx_end_from_acc = (bram_addr == 10'd15) ? 1'b1 : 1'b0; // 当地址达到15时结束

// BRAM地址生成（简单计数器）
reg [9:0] bram_addr_counter;
always @(posedge s00_axis_aclk or negedge s00_axis_aresetn) begin
    if (!s00_axis_aresetn) begin
        bram_addr_counter <= 0;
    end else if (phy_tx_start) begin
        bram_addr_counter <= bram_addr_counter + 1;
    end
end
assign bram_addr = bram_addr_counter;

// 监视器：显示关键信号
always @(posedge s00_axis_aclk) begin
    if (tx_bit_intf_acc_ask_data_from_s_axis) begin
        $display("Time %0t: TX bit interface requesting data, FIFO data count: %0d", 
                 $time, s_axis_fifo_data_count0);
    end
    
    if (phy_tx_start) begin
        $display("Time %0t: PHY TX started, BRAM data: 0x%016h", $time, data_to_acc);
    end
end

// 波形记录
initial begin
    $dumpfile("tb_tx_system.vcd");
    $dumpvars(0, tb_tx_system);
end

endmodule
```

## 工作原理解释

### 1. 系统架构
```
AXI Stream输入 → tx_intf_s_axis (FIFO缓冲) → tx_bit_intf (数据处理) → BRAM输出
```

### 2. 关键信号说明

**AXI Stream接口信号：**
- `s00_axis_tdata`：64位数据输入
- `s00_axis_tvalid`：数据有效信号
- `s00_axis_tready`：模块就绪信号
- `s00_axis_tlast`：数据包结束标志

**FIFO控制信号：**
- `s_axis_emptyn_to_acc`：FIFO非空指示
- `tx_bit_intf_acc_ask_data_from_s_axis`：数据请求信号
- `s_axis_data_to_acc`：FIFO输出数据

**TX控制信号：**
- `slice_en`：队列使能控制
- `backoff_done`：CSMA/CA退避完成
- `tx_control_state_idle`：TX状态机空闲
- `phy_tx_start`：PHY传输启动

### 3. 数据处理流程

1. **AXI数据接收**：通过AXI Stream接口接收数据到FIFO
2. **FIFO缓冲**：4个独立的FIFO对应不同优先级队列
3. **TX调度**：基于优先级和CSMA状态调度数据传输
4. **PHY头生成**：根据配置生成802.11 PHY头部
5. **BRAM写入**：处理后的数据写入BRAM供后续模块使用

### 4. 配置参数

- **DMA符号数**：控制单次传输的数据量
- **PHY头配置**：设置数据速率、长度、调制方式等
- **A-MPDU配置**：聚合帧相关参数
- **CTS保护**：RTS/CTS机制配置

这个测试平台完整模拟了WiFi TX数据通路的工作过程，可以验证从AXI数据输入到BRAM数据输出的完整流程。

# 接收模块仿真扩展

## 增加的测试平台代码

```verilog
// RX接口信号
wire [C_S00_AXIS_TDATA_WIDTH-1:0] rx_data_from_acc;
wire rx_acc_data_ready;
wire rx_fulln_to_acc;
wire [MAX_BIT_NUM_DMA_SYMBOL-1:0] rx_data_count;

// RX AXI Stream输出信号
wire rx_m_axis_tvalid;
wire [C_S00_AXIS_TDATA_WIDTH-1:0] rx_m_axis_tdata;
wire [(C_S00_AXIS_TDATA_WIDTH/8)-1:0] rx_m_axis_tstrb;
wire rx_m_axis_tlast;
reg rx_m_axis_tready;

// RX控制信号
reg rx_start_1trans;
reg [4:0] rx_start_count_cfg;
reg [MAX_BIT_NUM_DMA_SYMBOL-1:0] rx_m_axis_num_dma_symbol;
reg rx_endless_mode;

// RX模块实例化
rx_intf_m_axis #(
    .WAIT_COUNT_BITS(5),
    .MAX_NUM_DMA_SYMBOL(MAX_NUM_DMA_SYMBOL),
    .MAX_BIT_NUM_DMA_SYMBOL(MAX_BIT_NUM_DMA_SYMBOL),
    .C_M_AXIS_TDATA_WIDTH(C_S00_AXIS_TDATA_WIDTH)
) rx_intf_m_axis_i (
    .endless_mode(rx_endless_mode),
    .START_COUNT_CFG(rx_start_count_cfg),
    .M_AXIS_NUM_DMA_SYMBOL(rx_m_axis_num_dma_symbol),
    .start_1trans(rx_start_1trans),
    .DATA_FROM_ACC(rx_data_from_acc),
    .ACC_DATA_READY(rx_acc_data_ready),
    .data_count(rx_data_count),
    .FULLN_TO_ACC(rx_fulln_to_acc),
    .M_AXIS_ACLK(s00_axis_aclk),
    .M_AXIS_ARESETN(s00_axis_aresetn),
    .M_AXIS_TVALID(rx_m_axis_tvalid),
    .M_AXIS_TDATA(rx_m_axis_tdata),
    .M_AXIS_TSTRB(rx_m_axis_tstrb),
    .M_AXIS_TLAST(rx_m_axis_tlast),
    .M_AXIS_TREADY(rx_m_axis_tready)
);

// 连接TX输出到RX输入
assign rx_data_from_acc = data_to_acc; // 将TX的BRAM输出连接到RX的输入

// RX数据就绪信号生成 - 模拟ACC数据就绪
reg [2:0] data_ready_counter;
always @(posedge s00_axis_aclk or negedge s00_axis_aresetn) begin
    if (!s00_axis_aresetn) begin
        data_ready_counter <= 0;
        rx_acc_data_ready <= 0;
    end else begin
        // 在TX开始后延迟几个周期产生数据就绪信号
        if (phy_tx_start) begin
            data_ready_counter <= 3'b001;
            rx_acc_data_ready <= 0;
        end else if (data_ready_counter != 0) begin
            data_ready_counter <= data_ready_counter + 1;
            rx_acc_data_ready <= (data_ready_counter >= 3'b010) ? 1 : 0;
        end else begin
            rx_acc_data_ready <= 0;
        end
    end
end

// RX控制信号初始化
initial begin
    rx_start_1trans = 0;
    rx_start_count_cfg = 5'd2;    // 等待2个时钟周期
    rx_m_axis_num_dma_symbol = 8; // 接收8个DMA符号
    rx_endless_mode = 0;          // 非无限模式
    rx_m_axis_tready = 1;         // 始终准备好接收数据
end

// RX启动控制
reg rx_start_delay;
always @(posedge s00_axis_aclk or negedge s00_axis_aresetn) begin
    if (!s00_axis_aresetn) begin
        rx_start_delay <= 0;
        rx_start_1trans <= 0;
    end else begin
        // 在TX开始后延迟启动RX传输
        rx_start_delay <= phy_tx_start;
        if (rx_start_delay && !phy_tx_start) begin
            rx_start_1trans <= 1;
            #20; // 保持一个周期
            rx_start_1trans <= 0;
        end
    end
end

// RX数据监控
integer rx_data_received;
initial rx_data_received = 0;

always @(posedge s00_axis_aclk) begin
    if (rx_m_axis_tvalid && rx_m_axis_tready) begin
        rx_data_received <= rx_data_received + 1;
        $display("Time %0t: RX received data[%0d]: 0x%016h, TLAST: %b", 
                 $time, rx_data_received, rx_m_axis_tdata, rx_m_axis_tlast);
        
        if (rx_m_axis_tlast) begin
            $display("Time %0t: RX transmission completed, total packets: %0d", 
                     $time, rx_data_received + 1);
        end
    end
end

// RX FIFO状态监控
always @(posedge s00_axis_aclk) begin
    if (rx_data_count > 0) begin
        $display("Time %0t: RX FIFO data count: %0d", $time, rx_data_count);
    end
end
```

## 工作原理解释

### 1. 数据流架构
```
TX BRAM输出 (data_to_acc) → RX FIFO输入 → AXI Stream输出 (rx_m_axis_tdata)
```

### 2. 关键信号说明

**RX输入接口：**
- `rx_data_from_acc`：来自ACC的64位数据输入
- `rx_acc_data_ready`：ACC数据就绪信号，控制FIFO写入
- `rx_fulln_to_acc`：FIFO非满指示，反压信号

**RX控制信号：**
- `rx_start_1trans`：启动单次传输脉冲
- `rx_start_count_cfg`：启动前等待周期数
- `rx_m_axis_num_dma_symbol`：单次传输的DMA符号数量
- `rx_endless_mode`：无限传输模式使能

**AXI Stream输出：**
- `rx_m_axis_tvalid`：输出数据有效
- `rx_m_axis_tdata`：64位输出数据
- `rx_m_axis_tlast`：数据包结束标志
- `rx_m_axis_tready`：下游模块就绪信号

### 3. 状态机工作流程

接收模块包含3个主要状态：

1. **IDLE状态**：等待启动信号
2. **INIT_COUNTER状态**：初始化计数器，等待配置的周期数
3. **SEND_STREAM状态**：从FIFO读取数据并通过AXI Stream输出

### 4. FIFO控制逻辑

- **写入控制**：当`rx_acc_data_ready`为高且FIFO非满时写入数据
- **读取控制**：当`rx_m_axis_tvalid`和`rx_m_axis_tready`同时为高时读取数据
- **反压机制**：通过`rx_fulln_to_acc`信号防止FIFO溢出

### 5. 时序控制

- **启动延迟**：通过`rx_start_count_cfg`配置启动前的等待周期
- **传输长度**：通过`rx_m_axis_num_dma_symbol`控制单次传输的数据量
- **结束标志**：当读取指针达到配置的DMA符号数时产生TLAST信号

这个扩展实现了完整的接收通路，可以验证从TX模块输出到RX模块AXI Stream输出的完整数据流。