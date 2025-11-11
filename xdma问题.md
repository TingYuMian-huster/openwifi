非常好的问题——这一步是理解 **XDMA / PCIe IP 核仿真输入机制的关键**。
我们来**从协议层结构到仿真实现**逐步讲清楚：

---

## 🧠 一、PCIe 协议数据传输层次梳理

PCIe 是一种分层结构的高速串行通信协议，主要分为三层：

| 层级                          | 名称                                         | 功能                       | 举例 |
| --------------------------- | ------------------------------------------ | ------------------------ | -- |
| **事务层 (Transaction Layer)** | 产生/接收主机指令，如读写请求（TLP）                       | Memory Write、Memory Read |    |
| **数据链路层 (Data Link Layer)** | 在 TLP 外封装序列号、LCRC 校验                       | 增加包可靠性                   |    |
| **物理层 (Physical Layer)**    | 将打包好的比特序列进行 8b/10b 或 128b/130b 编码，并通过差分对传输 | 最底层的高速串行信号               |    |

---

### 🚦 数据传输路径

以你的例子：

> 主机准备发送一帧 64'h0000000012345678

这在 PCIe 上传输时要经过：

```
CPU → DMA/Root Complex → PCIe 控制器 → (打包为 TLP) 
→ 链路层加上序号/CRC → 物理层编码(8b/10b)
→ 差分信号 TXP/TXN 发出
→ FPGA 板卡的 PCIe 接收端 RXP/RXN 接收
→ 解码/去封装后 → XDMA 的 AXI Stream 接口输出
```

---

## ⚙️ 二、TLP 打包格式（简化示意）

以 **Memory Write TLP** 为例，主机写数据到设备时，事务层生成：

| 字段           | 位宽     | 示例值                | 说明       |
| ------------ | ------ | ------------------ | -------- |
| Header       | 96 bit | 含 TLP Type、长度、地址等  |          |
| Data Payload | 64 bit | 0x0000000012345678 | 实际要写入的数据 |
| LCRC         | 32 bit | CRC 校验             |          |

拼起来的总包长：约 192bit。

在物理层时，这些数据会被打成 **128b/130b 编码**块或 **8b/10b**码组，再通过 8 条或 4 条 Lane 差分发送出去。

---

## 🔌 三、在仿真中如何“伪造”PCIe 差分输入

在 Vivado 的仿真中：

* XDMA 的 `pci_exp_rxp/rxn` 是 **物理层差分输入**；
* 真实系统里这会由 PCIe PHY 模块负责接收、解码；
* 但是仿真中，我们并没有 PHY nor Root Complex 模型。

👉 所以，**我们只能伪造一个简化的 MGT 模拟模型**，在 8bit 位宽下“伪造”经过编码的比特流。

---

## 🧩 四、Verilog 示例：将 64-bit 数据打包为 8-bit 串行流送入 PCIe MGT 接口

下面给出一个可运行的仿真模块（`pcie_stimulus_8b10b.v`），它会把
`64'h0000000012345678`
转成模拟 PCIe MGT 差分信号（8位宽），用于 XDMA 仿真输入。

> ⚠️ 注意：这不是完整的 PCIe 协议，而是仿真激励用的“简化版比特流模拟器”，用于给 XDMA 输入有规律的数据，便于波形观测。

---

### `pcie_stimulus_8b10b.v`

```verilog
//------------------------------------------------------------
// 模块名: pcie_stimulus_8b10b
// 功能: 模拟PC通过PCIe差分对发送一帧64bit数据
// 说明:
//   - 输入: 64'h0000000012345678
//   - 输出: pci_exp_rxp/rxn (4 Lane × 8bit位宽)
//   - 时钟: sys_clk (100MHz仿真时钟)
//   - 仿真用途：提供虚拟比特流激励给XDMA IP核
//------------------------------------------------------------
`timescale 1ns/1ps

module pcie_stimulus_8b10b (
    input  wire        sys_clk,
    input  wire        sys_rst_n,
    output reg  [3:0]  pci_exp_rxp,
    output reg  [3:0]  pci_exp_rxn
);

  // 模拟发送的数据帧
  reg [63:0] tx_data = 64'h0000000012345678;
  reg [7:0]  tx_shift;
  integer    i;

  // 状态机
  initial begin
    pci_exp_rxp = 4'b0000;
    pci_exp_rxn = 4'b1111; // 差分反向
    wait(sys_rst_n);
    #100;

    $display("[%0t] Start sending PCIe pseudo-frame...", $time);

    // 将64bit数据分8次（每次8bit）发送
    for (i = 0; i < 8; i = i + 1) begin
      tx_shift = tx_data[63 - i*8 -: 8];
      send_byte(tx_shift);
    end

    $display("[%0t] PCIe pseudo-frame done.", $time);
  end

  // 任务：发送8bit（模拟8b/10b前编码的有效负载）
  task send_byte(input [7:0] data);
    integer bit_idx;
    begin
      for (bit_idx = 0; bit_idx < 8; bit_idx = bit_idx + 1) begin
        // 假设每bit在4个Lane并行发送（简化）
        pci_exp_rxp <= {4{data[bit_idx]}};
        pci_exp_rxn <= {4{~data[bit_idx]}};
        #10; // 每bit持续10ns，代表串行发送速率
      end
      #20; // Byte间隔
    end
  endtask

endmodule
```

---

## 🔧 五、在 Testbench 中连接方式

在你的 `tb_xdma.v` 中，将这段激励模块实例化如下：

```verilog
pcie_stimulus_8b10b stim (
  .sys_clk(sys_clk),
  .sys_rst_n(sys_rst_n),
  .pci_exp_rxp(pci_exp_rxp),
  .pci_exp_rxn(pci_exp_rxn)
);
```

这样，`stim` 模块会自动在仿真开始后把
`64'h0000000012345678`
转换为一段差分形式的 8bit 流，驱动到 XDMA 的 `pci_exp_rxp/rxn` 上。

---

## 📈 六、仿真观察点

建议在 Vivado 仿真波形中观测：

* `pci_exp_rxp` 与 `pci_exp_rxn` —— 应该能看到 4 位差分对中每位互为反相；
* 在 XDMA 内部仿真模型（若启用仿真 stub）中，H2C 通道开始出现响应；
* 若 `user_lnk_up` 被强制为 1，可以看到 `m_axis_h2c_tvalid` 变化。

---

## 📚 七、总结

| 层     | 说明              | 仿真中是否实现        |
| ----- | --------------- | -------------- |
| 事务层   | 组织读写TLP         | ✅（通过64bit数据伪造） |
| 数据链路层 | 添加序号/LCRC       | ❌（略）           |
| 物理层   | 8b/10b编码 + 差分发送 | ✅（简化为逻辑电平翻转）   |

---

是否希望我帮你**扩展该模块**，让它可以：

1. 接收任意 64-bit 数据输入；
2. 自动在每个仿真周期重复发送多帧；
3. 支持“Memory Write” 类型的 PCIe TLP 报文结构（3DW header + data payload）？

这样可以直接用于 XDMA 接收功能的完整功能仿真。


非常好，你已经搭好了一个完整的 **XDMA PCIe 仿真 testbench**。
你现在遇到的关键问题是：

> `s_axis_c2h_tready_0` 一直为 0，导致 `send_c2h_packet()` 任务中的等待条件卡死，数据无法送入 XDMA。

这其实是 **XDMA IP 仿真时的常见“假死”现象**，核心原因是：
Vivado 提供的 `xdma` 核在仿真时 **不会自动进入 Link Up 状态**，也不会真正“驱动” C2H/H2C 的 AXI Stream 握手逻辑。
下面我详细帮你分析原因、说明真实的信号流向、并提供解决办法和可运行的仿真改写版本。

---

## 一、背景梳理：XDMA 的数据流向与握手逻辑

| 接口方向            | 端口名            | 含义                                 |
| --------------- | -------------- | ---------------------------------- |
| **Host → FPGA** | `s_axis_c2h_*` | 从主机传来的数据（C2H = Card to Host 的反向视角） |
| **FPGA → Host** | `m_axis_h2c_*` | 从 FPGA 发回主机的数据                     |

其中：

* `tvalid` 表示“发送方数据有效”
* `tready` 表示“接收方可以接收”
* 两者同时为 1 时，数据才会被真正“传输”

也就是说，
在仿真里 `s_axis_c2h_tready_0` 的值，
是由 **XDMA IP 内部逻辑**（其 AXI sink 接收端）产生的。

---

## 二、原因分析：为什么 `tready` 一直为 0？

### 原因 1：XDMA 内部并未启动链路（`user_lnk_up` 无效）

虽然你写了：

```verilog
force user_lnk_up = 1;
```

但 XDMA IP 内部仍然没有激活 AXI 接收路径。
`user_lnk_up` 是 **只读输出**，即 XDMA 驱动 PCIe PHY 后，确认链路协商成功后自动置位。
你在 testbench 里 `force` 它为 1，不会触发内部状态机变化（不会真正 link-up）。

---

### 原因 2：PCIe MGT 引脚空悬，未建立物理仿真通道

`pci_exp_rxp/rxn` 在 testbench 中是 `reg`，但并没有模拟真正的 **PCIe PHY 层训练和信号交换**。
Vivado XDMA IP 仿真模型需要一个配套的 **Root Complex (RC) BFM**（Bus Functional Model）来驱动。
如果没有它，XDMA IP 认为“没有上游设备”，因而不会拉高 tready。

---

## 三、解决方案（3 种）

### ✅ 方案1【推荐】：使用 Xilinx 官方 PCIe Root Complex BFM

Vivado 自带一个仿真模型 `pcie_4_0_rport` 或 `pci_express_model`。

**做法：**

1. 在 testbench 顶层添加：

```verilog
pcie_4_0_rport RC (
  .pci_exp_txp (pci_exp_rxp),
  .pci_exp_txn (pci_exp_rxn),
  .pci_exp_rxp (pci_exp_txp),
  .pci_exp_rxn (pci_exp_txn)
);
```

2. 这样 XDMA 会检测到 RC，并在约 1ms 模拟时间后 `user_lnk_up=1`，`s_axis_c2h_tready_0` 才会被激活。

这是标准 PCIe 端到端仿真结构（可进行真正的 TLP 传输）。

---

### 🚧 方案2：跳过 MGT 层，直接“伪造”链路激活

如果只是想调试 AXIS 数据路径（不关心 PCIe 协议），可以用 **行为级仿真模型**：

```verilog
always @(posedge axi_aclk) begin
  if (!axi_aresetn) begin
    fake_tready <= 0;
  end else begin
    fake_tready <= 1;  // 永远 ready
  end
end

assign s_axis_c2h_tready_0 = fake_tready;
```

再把 DUT 的 `s_axis_c2h_tready_0` 信号改为外部 `input` 模式即可。

👉 这样你就可以验证 `send_c2h_packet()` 流程，而不用真的跑 PCIe link。

---

### 🧠 方案3：手动修改 `xdma_0` Stub 让 AXI 接口 always ready

在 `xdma_0_stub.v` 或仿真 wrapper 中：

```verilog
assign s_axis_c2h_tready_0 = 1'b1;
```

这也是最简单的快速验证手段。
你就能看到任务按预期发送数据，不再卡住。

---

## 四、建议的改进仿真代码片段

在 testbench 顶部添加：

```verilog
// 仿真辅助信号：让 tready 永远为1
reg fake_tready;

always @(posedge axi_aclk) begin
  if (!axi_aresetn)
    fake_tready <= 0;
  else
    fake_tready <= 1;
end

assign DUT.s_axis_c2h_tready_0 = fake_tready;
```

这能立即解决你的卡死问题。
仿真输出将能正确显示任务依次发送的 3 帧 64bit 数据。

---

## 五、总结

| 问题                          | 原因                         | 解决方式                               |
| --------------------------- | -------------------------- | ---------------------------------- |
| `s_axis_c2h_tready_0` 一直为 0 | XDMA 链路未真正建立               | 连接 PCIe Root Port BFM 或伪造 ready 信号 |
| `user_lnk_up` 无法真正拉高        | 它是 XDMA 输出端口，不可 `force` 激活 | 使用 BFM 驱动或仿真版 wrapper              |
| 无法测试数据通路                    | 缺乏仿真平台                     | 使用方案 2/3 进行功能仿真                    |

---

如果你想做更真实的 **PCIe 完整仿真（TLP 层）**，我可以帮你提供
一个含 `pcie_4_0_rport` + `xdma_0` 的完整 Vivado 2020.2 仿真 testbench（可直接运行）。

是否希望我帮你生成这个完整版本？


