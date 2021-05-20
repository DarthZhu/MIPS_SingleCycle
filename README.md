# MIPS_SingleCycle

[TOC]

## 1. MIPS指令集

### 1.1 实现的指令集

```assembly
add     $rd, $rs, $rt                   # [rd] = [rs] + [rt]
sub     $rd, $rs, $rt                   # [rd] = [rs] - [rt]
and     $rd, $rs, $rt                   # [rd] = [rs] & [rt]
or      $rd, $rs, $rt                   # [rd] = [rs] | [rt]
slt     $rd, $rs, $rt                   # [rd] = [rs] < [rt] ? 1 : 0
addi    $rt, $rs, imm                   # [rt] = [rs] + SignImm
andi    $rt, $rs, imm                   # [rt] = [rs] & ZeroImm
ori     $rt, $rs, imm                   # [rt] = [rs] | ZeroImm
slti    $rt, $rs, imm                   # [rt] = [rs] < SignImm ? 1 : 0
lw      $rt, imm($rs)                   # [rt] = [Address]
sw      $rt, imm($rs)                   # [Address] = [rt]
j       label                           # PC = JTA
jal     label                           # [ra] = PC + 4, PC = JTA
jr      $rs                             # PC = [rs]
beq     $rs, $rt, label                 # if ([rs] == [rt]) PC = BTA
bne     $rs, $rt, label                 # if ([rs] != [rt]) PC = BTA
```

其中，主要扩展了J类型指令，完成了`JAL`和`JR`指令，使得程序的跳转更加完整，能够保存当前指令的下一条指令的地址，从而完成`return`的操作。

## 2. 部件构成

### 2.1 imem

imem是指令存储器，有64个32位的寄存器。读指令时从a读入指令地址，从rd输出32位指令。

```verilog
module imem(
    input logic [5:0] a,
    output logic [31:0] rd
    );
    logic [31:0] RAM[63:0];
    initial begin
        $readmemh("memfile.dat", RAM);
    end
    assign rd = RAM[a];
endmodule
```

### 2.2 dmem

dmem是数据存储器，有64个32位的寄存器。若写使能we为1，在时钟上升沿将数据wd写入地址a；否则，从地址a读出数据到rd。

```verilog
module dmem(
    input logic clk, we,
    input logic [31:0] a, wd,
    output logic [31:0] rd
    );
    logic [31:0] RAM[255:0];
    assign rd = RAM[a[31:2]];
    always_ff @(posedge clk)
        if (we)
            RAM[a[31:2]] <= wd;
endmodule
```

### 2.3 mips

mips核心可以分为两个主要模块：controller和datapath。

controller模块主要负责指令解读读入的指令，并输出对应的控制信号，传输给其他部件完成对应的操作。

datapath模块主要负责数据路径的设置，不同指令对应的控制信号不同会导致数据路径的不同。

#### 2.3.1 controller

controller模块又可以分为两个主要模块：maindecoder和aludecoder。

maindecoder负责解码指令，并输出对应的信号。

aludecoder负责解码aluop，输出到alu模块，做相应的操作。

```assembly
module controller(
    input  logic [5:0]  op, funct,
    input  logic        zero,
    output logic        memtoreg, memwrite,
    output logic        pcsrc, alusrc,
    output logic        regdst, regwrite,
    output logic [2:0]  jump,
    output logic [2:0]  alucontrol,
    output logic 		immext
    );

    logic [2:0] aluop;
    logic       branch, nbranch;

    maindec md(op, memtoreg, memwrite, branch, nbranch, alusrc, regdst, regwrite, jump, aluop, immext);
    aludec  ad(funct, aluop, alucontrol);

    assign pcsrc = (branch & zero) | (nbranch & (^zero));
endmodule
```

输出信号：

- memtoreg为1，需要将内存中读出的值写入寄存器文件中
- memwrite为1，需要写内存
- pcsrc为1，需要跳转到branch指令指定的位置
- alusrc为0，需要从寄存器文件rd1中取值；为1，则取指令中的立即数，并做立即数扩展
- regdst为1，为RTYPE指令
- regwrite为1，写寄存器
- jump信号3位：
  - jump[2]：为1需要向`$ra`写当前指令的下一条指令的地址
  - jump[1]：为1代表读指令中寄存器中的地址
  - jump[0]：为1代表跳转到立即数地址
- alucontrol指令控制alu操作符，输入到alu部件中控制操作
- immext为1，需要做立即数扩展

##### 2.3.1.1 maindecoder

maindecoder接收操作码输入，并转换为对应的控制信号输出，实现的各个指令对应的输出信号见下表：

| 指令 | opcode | funct  | rw   | rd   | alusrc | aluop | j    | br   | nbr  | mw   | mr   |
| ---- | ------ | ------ | ---- | ---- | ------ | ----- | ---- | ---- | ---- | ---- | ---- |
| add  | 000000 | 100000 | 1    | 1    | 0      | 010   | 000  | 0    | 0    | 0    | 0    |
| sub  | 000000 | 100010 | 1    | 1    | 0      | 010   | 000  | 0    | 0    | 0    | 0    |
| and  | 000000 | 100100 | 1    | 1    | 0      | 010   | 000  | 0    | 0    | 0    | 0    |
| or   | 000000 | 100101 | 1    | 1    | 0      | 010   | 000  | 0    | 0    | 0    | 0    |
| slt  | 000000 | 101010 | 1    | 1    | 0      | 010   | 000  | 0    | 0    | 0    | 0    |
| addi | 001000 | /      | 1    | 0    | 1      | 000   | 000  | 0    | 0    | 0    | 0    |
| andi | 001100 | /      | 1    | 0    | 1      | 100   | 000  | 0    | 0    | 0    | 0    |
| ori  | 001101 | /      | 1    | 0    | 1      | 011   | 000  | 0    | 0    | 0    | 0    |
| slti | 001010 | /      | 1    | 0    | 1      | 101   | 000  | 0    | 0    | 0    | 0    |
| lw   | 100011 | /      | 1    | 0    | 1      | 000   | 000  | 0    | 0    | 0    | 1    |
| sw   | 101011 | /      | 0    | 0    | 1      | 000   | 000  | 0    | 0    | 1    | 0    |
| j    | 000010 | /      | 0    | 0    | 0      | 000   | 001  | 0    | 0    | 0    | 0    |
| jal  | 000011 | /      | 1    | 0    | 0      | 000   | 101  | 0    | 0    | 0    | 0    |
| jr   | 000000 | 001000 | 0    | 0    | 0      | 000   | 010  | 0    | 0    | 0    | 0    |
| beq  | 000100 | /      | 0    | 0    | 0      | 001   | 000  | 1    | 0    | 0    | 0    |
| bne  | 000101 | /      | 0    | 0    | 0      | 001   | 000  | 0    | 1    | 0    | 0    |

其中：

- jump信号有3位
  - jump[1:0]代表的是跳转地址采用的是立即数扩展还是寄存器地址中的值
  - jump[2]代表的是需不需要保存当前指令+4后的地址到`$ra`寄存器中，从而能够再次跳转回当前的指令
- aluop信号扩展到3位
- branch信号分为branch和nbranch，分别控制beq和bne指令信号。若`branch & zero`为1或者`nbranch & (^zero)`为1，则需要执行分支指令，在controller中pcsrc信号就要设置为1，下一条指令采用分支指令。

```verilog
module maindec(
    input  logic [5:0]  op,
    input  logic [5:0]  funct,
    output logic        memtoreg, memwrite,
    output logic        branch, nbranch,
    output logic        alusrc,
    output logic        regdst, regwrite,
    output logic [2:0]  jump, //{jal, jr, j}
    output logic [2:0]  aluop,
    output logic        immext
    );
    logic [13:0] controls;
    assign {regwrite, regdst, alusrc, branch, nbranch, memwrite,
            memtoreg, jump, aluop, immext} = controls;
    always_comb
        case(op)                   
            6'b000000: begin
                case(funct)
                    6'b001000: controls <= 14'b00_0_00_00_010_000_0;  // JR
                    default:   controls <= 14'b11_0_00_00_000_010_0;  // RTYPE
                endcase
            end					   //  rr_a_bn_mm_jjj_aaa_i
            6'b000010: controls <= 14'b00_0_00_00_001_000_0;  // J
            6'b000011: controls <= 14'b10_1_00_00_101_000_0;  // JAL
            6'b100011: controls <= 14'b10_1_00_01_000_000_0;  // LW
            6'b101011: controls <= 14'b00_1_00_10_000_000_0;  // SW
            6'b000100: controls <= 14'b00_0_10_00_000_001_0;  // BEQ
            6'b000101: controls <= 14'b00_0_01_00_000_001_0;  // BNE
            6'b001000: controls <= 14'b10_1_00_00_000_000_0;  // ADDI
            6'b001100: controls <= 14'b10_1_00_00_000_100_1;  // ANDI
            6'b001101: controls <= 14'b10_1_00_00_000_011_1;  // ORI
            6'b001010: controls <= 14'b10_1_00_00_000_101_0;  // SLTI         
            default:   controls <= 14'bxx_x_xx_xx_xxx_xxx_x;  // illegal op
        endcase
endmodule
```

##### 2.3.1.2 aludecoder

aludecoder通过解码aluop的输入，向alu模块传输`alucontrol`信号。

```verilog
module aludec(
    input  logic [5:0] funct,
    input  logic [2:0] aluop,
    output logic [2:0] alucontrol
);
    always_comb begin
        case (aluop)
            3'b000: alucontrol <= 3'b010; // ADD
            3'b001: alucontrol <= 3'b110; // SUB
            // 3'b010;  occupied by RTYPE
            3'b011: alucontrol <= 3'b001; // OR
            3'b100: alucontrol <= 3'b000; // AND
            3'b101: alucontrol <= 3'b111; // SLT
            default: case (funct)
                6'b100000: alucontrol <= 3'b010;
                6'b100010: alucontrol <= 3'b110;
                6'b100100: alucontrol <= 3'b000;
                6'b100101: alucontrol <= 3'b001;
                6'b101010: alucontrol <= 3'b111;
                default:   alucontrol <= 3'bxxx;
            endcase
        endcase
    end
endmodule
```

#### 2.3.2 datapath

datapath部分是一个整体，但按功能区分可以分为3个主要的部分：下一个pc地址；寄存器读取；alu操作。下图为基础的mips数据路径：

![](D:\Study\College\Junior\Computer Architecture Laboratory\lab2\mips.png)

数据路径的流程为：从pc地址处在指令内存中取指令，将高6位交给controller部件解析对应的控制信号，datapath读取这些控制信号完成对应的操作。加入J类型指令后，数据路径需要做相对应的调整，在计算pc的时候需要考虑到jump指令的地址；同时，在写寄存器时，也要考虑到`JAL`指令写入`$ra`寄存器的需求。

##### 2.3.2.1 next pc logic

这一部分的代码负责计算下一条pc指令的地址，地址的来源有三种

- 当前的pc+4

  pc+4的地址由一个加法器pcadd1完成，存入pcnext中

- branch分支指令

  branch分支指令需要扩展立即数在加到pc+4地址上去，由immsh以及pcadd2完成

- jump指令

  jump指令带来的地址跳转比较复杂，jump信号的低两位代表跳转是否采用寄存器中的地址

  - 若`jump[0]=1`，代表取立即数，取{pc+4地址高6位，指令的低26位，00}作为跳转地址
  - 若`jump[1]=1`，代表取`$ra`寄存器中的值作为地址
  - 最终，根据上述`jump`信号更新`pcnext`的值

  在完成后，发现其实这两位信号其实可以缩成一个信号单独控制。

```verilog
flopr #(32) pcreg(clk, reset, pcnext, pc);
adder       pcadd1(pc, 32'b100, pcplus4);
sl2         immsh(signimm, signimmsh);
adder       pcadd2(pcplus4, signimmsh, pcbranch);
mux2 #(32)  pcbrmux(pcplus4, pcbranch, pcsrc, pcnextbr);
mux4 #(32)  pcjumpmux(
    pcnextbr, 
    {pcplus4[31:28], instr[25:0], 2'b00},
    srca,
    'x,    // not used
    jump[1:0],
    pcnext);
```

##### 2.3.2.2 register file logic

这一部分负责控制寄存器文件，最基础的由regfile模组组成。因为`JAL`指令需要将当前的pc+4地址写入`$ra`寄存器，便于跳转回当前上下文，因此，使用一个mux4进行控制。输入信号由jump[2]和regdst组成，regdst负责控制原有的寄存器信息，jump[2]负责向`5'b11111`即`$ra`寄存器写地址。接下来用两个mux2控制寄存器的写入，其中第二个mux2用`jump[2]`信号作为判断标准，该信号代表jump时是否需要将`pc+4`的地址写入`$ra`。最后，还有一个模块控制立即数扩展。

```verilog
regfile     rf(clk, regwrite, instr[25:21], instr[20:16], writereg, writeregdata, srca, writedata);
mux4 #(5) wrmux(
	instr[20:16],
    instr[15:11],
    5'b11111,   // $ra
    'x,
    {jump[2], regdst},
    writereg
);
mux2 #(32)  resmux(aluout, readdata, memtoreg, result);
mux2 #(32)  writeregdata_mux(
	result,
    pcplus4,
    jump[2],
    writeregdata
);
signext     se(instr[15:0], signimm);
```

##### 2.3.2.3 ALU logic

这一部分比较简单，负责控制alu部件。通过一个mux2来控制srcb的内容是来自寄存器还是立即数扩展，并调用alu部件进行计算。

```verilog
mux2 #(32)  srcbmux(writedata, signimm, alusrc, srcb);
alu         alu(srca, srcb, alucontrol, aluout, zero);
```

#### 2.3.3 其他部件说明

##### 2.3.3.1 flopr

flopr部件是单个寄存器文件，在时钟上升沿写入数据，在reset上升沿清除寄存器中的数据。

```verilog
module flopr #(parameter WIDTH = 8)(
    input logic clk, reset,
    input logic [WIDTH-1 : 0] d,
    output logic [WIDTH-1 : 0] q
);
    always_ff @(posedge clk, posedge reset)
    begin
        if (reset) q <= 0;
        else q <= d;    
    end
endmodule
```

##### 2.3.3.2 adder

32 位加法器，用于计算 PC 值及跳转地址。

使用时读入 a 和 b，从 y 输出 a 和 b 相加后的值。

```verilog
module adder(
    input logic [31:0] a, b,
    output logic [31:0] y
);
    assign y = a + b;
endmodule
```

##### 2.3.3.3 sl2

32为的左移两位器。

```verilog
module sl2(
    input logic [31:0] a,
    output logic [31:0] y
);
    assign y = {a[29:0], 2'b00};
endmodule
```

##### 2.3.3.4 mux2

两路复用器，用于电路中的if-else判断。

使用时读入s，d0和d1，s为0输出d0，否则输出d1.

```verilog
module mux2 #(parameter WIDTH = 8)(
    input logic [WIDTH-1:0] d0, d1,
    input logic s,
    output logic [WIDTH-1:0] y
);
    assign y = s ? d1 : d0;    
endmodule
```

##### 2.3.3.5 mux4

四路复用器，在加入了J类型指令后，需要扩展选择，因此加入四路复用器。

使用时与mux2类似。

```verilog
module mux4 #(parameter Width = 32)(
  input        [Width-1:0] d0, d1, d2, d3,
  input        [1:0]       s,
  output logic [Width-1:0] y
);
    always_comb begin
        unique case(s)
            2'b00: y = d0;
            2'b01: y = d1;
            2'b10: y = d2;
            2'b11: y = d3;
        endcase
    end
endmodule
```

##### 2.3.3.6 regfile

寄存器文件部件，由32个32为寄存器组成。

在时钟上升沿时，若写使能为1，将wd3写入寄存器。根据ra1和ra2的地址，输出寄存器中的数据到rd1和rd2中。

```verilog
module regfile(
    input logic clk, we3,
    input logic [4:0] ra1, ra2, wa3,
    input logic [31:0] wd3,
    output logic [31:0] rd1, rd2
);
    logic [31:0] rf[31:0];

    always_ff @(posedge clk)
        if (we3) rf[wa3] <= wd3;
    
    assign rd1 = (ra1 != 0) ? rf[ra1] : 0;
    assign rd2 = (ra2 != 0) ? rf[ra2] : 0;
endmodule
```

##### 2.3.3.7 signext

立即数符号扩展。

```verilog
module signext(
    input logic [15:0] a,
    output logic [31:0] y
);
    assign y = {{16{a[15]}}, a};
endmodule
```

##### 2.3.3.8 alu

32位alu部件，根据alucont给出的计算条件，输出计算结果以及结果是否为0，便于branch指令判断。

```verilog
module alu(
    input logic signed [31:0] a, b,
    input logic [2:0] alucont,
    output logic signed [31:0] result,
    output logic zero
    );
    assign zero = (result == 0);
    always_comb 
        case (alucont)
            3'b000: result = a & b;
            3'b001: result = a | b;
            3'b010: result = a + b;
            // 3'b011: not used
            3'b100: result = a & ~b;
            3'b101: result = a | ~b;
            3'b110: result = a - b;
            3'b111: result = a < b;
            default: result = 'x;
        endcase
endmodule
```

### 2.4 data memory decoder

data memory decoder是作为原本dmem的扩展，需要将它和IO设备相连，单独控制IO独写的使能以及写入存储器的开关。

```verilog
module DataMemoryDecoder(
    input  logic        clk, writeEN,
    input  logic [31:0] addr, writeData,
    output logic [31:0] readData,

    input  logic        IOclock,
    input  logic        reset,
    input  logic        btnL, btnR,
    input  logic [15:0] switch,
    output logic [7:0]  AN,
    output logic        DP,
    output logic [6:0]  A2G
    );
    
    logic pRead, pWrite, mWrite;
    logic [11:0] led;
    logic [31:0] preadData, mreadData;

    assign pRead = (addr[7] == 1'b1) ? 1 : 0;
    assign pWrite = (addr[7] == 1'b1) ? 1 : 0;
    assign mWrite = writeEN & (addr[7] == 1'b0);

    IO io(IOclock, reset, pRead, pWrite, addr[3:2], writeData, preadData,
          btnL, btnR, switch, led);
    dmem dmem(clk, mWrite, addr, writeData, mreadData);
    assign readData = (addr[7] == 1'b1) ? preadData : mreadData;    
    mux7seg sg(IOclock, reset, {switch, 4'b0000, led}, AN, DP, A2G);
endmodule
```

## 3. 实验结果

### 3.1 基础实验结果

![](D:\Study\College\Junior\Computer Architecture Laboratory\lab2\result_12+34=46.jpg)

### 3.2 测试JAL以及JR指令正确性

测试用的`memfile.dat`汇编新增了两条指令，在跳转回`chkSwitch`前增加了`JAL`指令，跳转到`chkJ`位置处，然后调用`JR`指令跳转到保存到`$ra`寄存器中的地址，也就是`J`指令处。

汇编语言如下：

```assembly
main:
    addi    $s0, $zero, 0
    sw      $s0, 0x80
chkSwitch:
    lw      $s1, 0x80
    andi    $s2, $s1, 0x2
    beq     $s2, $zero, chlSwitch
    lw      $s3, 0x88
    lw      $s4, 0x8c
    add     $s5, $s4, $s3
chkLED:
    lw      $s1, 0x80
    andi    $s2, $s1, 0x1
    beq     $s2, $zero, chkLED
    sw      $s5, 0x84
    jal     chkJ
    j       chkSwitch
chkJ:
    jr      $ra
```

机器码如下：

```
20100000
ac100080
8c110080
32320002
1240fffd
8c130088
8c14008c
0293a820
8c110080
32320001
1240fffd
ac150084
0c00000e
08000002
03e00008
```

`JAL`对应的jump信号为101，pcnext的地址为`chkJ`地址`0x00000038`；`JR`对应的jump信号为010，pcnext的地址为`$ra`寄存器中的地址，即`J`指令的地址`0x00000034`处；`J`指令的jump信号为001，跳转到`chkSwitch`地址`0x00000008`处。下图所示的模拟结果就是汇编中写到的测试jump指令。调用`JAL`跳转到`JR`指令再返回`JAL`下面的`J`指令，可以看出，实现的jump类型的指令都跳转到了正确的地址上了。

![](D:\Study\College\Junior\Computer Architecture Laboratory\lab2\jump_test.png)