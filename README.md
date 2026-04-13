# 0. 文件结构
```
tdc
├── tdc_channelbank
│   ├── tdc_channelbank_single / multi
│   │   ├── tdc_channel
│   │   │   ├── tdc_ringosc
│   │   │   ├── tdc_delayline
│   │   │   │   └── tdc_ordertaps
│   │   │   └── tdc_lbc
│   │   └── tdc_freqc
│   │       └── tdc_psync × 3
└── tdc_controller
    └── tdc_divider
```

# 1. 时序链上发生的动作
```
每次 delay line 被采样并形成新的寄存 tap 向量后，都会立即经过 ordertaps 和 reverse，然后送给 tdc_lbc。

CARRY4 延迟链
-> 两级 FDR 采样
-> 得到已寄存的 tap 向量
-> tdc_ordertaps 重排
-> reverse
-> taps_o
-> tdc_lbc 编码
```

## 1.1 tdc_ordertaps 重排
```
第一步：tdc_ordertaps

解决局部顺序错位，例如：

原始索引顺序：0 1 2 3 4 5 6 7
真实时间顺序：0 1 2 3 4 5 7 6

这一步是“排队纠错”。
```

## 1.2 reverse
```
第二步：reverse

把排好序的整串 bit 再整体翻转一下，比如：

排好序后：11111110
reverse 后：01111111

这一步是“统一方向”。
```

```
因为 reverse 做的不是“按值排序”，而是单纯把 bit 方向倒过来：

原来位置: 7 6 5 4 3 2 1 0
原来数值: 1 1 1 1 1 1 1 0

翻转后:
新位置:   7 6 5 4 3 2 1 0
新数值:   0 1 1 1 1 1 1 1

如果用代码思想表达，就是类似：
taps_o[k] = taps_rev_sorted[7-k];
对于 8 位来说：

taps_o[0] = sorted[7]
taps_o[1] = sorted[6]
...
taps_o[7] = sorted[0]
```

```
为什么还要 reverse
因为 tdc_lbc 对输入位序是有偏好的。
通常编码器会固定从某一端开始数连续的 1。

如果它的实现是“从 MSB 往 LSB 数前导 1”，那就希望输入长成：
11111110

如果它的实现等价于“另一端开始看”，那就可能更希望输入长成：
01111111

所以 reverse 的本质是：
把已经按真实时间顺序排好的向量，再变成适合编码器读取方向的位序。
```

# 2. tdc_controller
```
它在做三件事：

上电后先把每个通道校准一遍
运行过程中持续在线更新 LUT
在自动模式和 freeze/debug 模式之间切换控制权

真正做时间采样的是 tdc_delayline + tdc_lbc + tdc_channel。
tdc_controller 不直接采样，它只负责安排：
现在采谁
现在是不是喂校准信号
histogram 要不要清零/更新
什么时候测 ring oscillator 频率
什么时候把新 LUT 写回去
什么时候切到下一个通道
```

## 2.1 它上电后先做什么
```
上电后，controller 不会立刻宣布系统 ready。
它会先进入一轮 启动校准。

这轮启动校准的核心目标是两件事：

第一件事：为每个通道建立 histogram
也就是统计：
raw=0 出现几次
raw=1 出现几次
raw=2 出现几次
…
这样就知道每个 bin 的相对宽度。

第二件事：保存每个通道的参考频率 sfreq
“刚完成初始校准时，这个通道的 ring oscillator 跑多快”
```

## 2.2 启动校准阶段
```
第一步：选中一个通道
如果是多通道，controller 一次只校一个通道。

第二步：清空这个通道对应的 histogram
把这一通道所有 raw bin 的统计计数都清 0。

第三步：切到校准输入
这时 controller 会让 calib_sel 选中 calib_i，不是正常的 signal_i。
也就是说，此时通道前端测的不是外部事件，而是专门的校准脉冲。

第四步：等事件出现，然后更新 histogram
每次校准脉冲经过 delay chain，被编码成一个 raw。
controller 看到 c_detect_i=1 后，就用 c_raw_i 去更新：
hist[c_raw_i] += 1
重复很多次，直到样本收集够。

第五步：测一次 ring oscillator 频率
把这时测到的频率存为这个通道的 sfreq。

第六步：切到下一个通道
重复上面的过程，直到所有通道都做完。
这就是启动校准。
```

## 2.3 启动校准完成后做什么
```
启动校准完成后，controller 进入 在线校准。

在线校准不是重新从头统计一次所有 histogram，而是：
在已有 histogram 的基础上，结合当前 ring oscillator 频率，重算 LUT。

这一步的核心公式就是你前面已经看到的：
LUT[k] = acc[k] * sfreq / freq

其中：
acc[k]：histogram 的累计值
sfreq：之前保存的参考频率
freq：当前实时测得频率
```

## 2.4 在线校准阶段
```
第一步：选中一个通道
还是一次只处理一个通道。

第二步：测当前通道的 ring oscillator 频率
得到当前 freq。

第三步：顺序读 histogram
controller 从地址 0 开始，把这个通道所有 histogram bin 读一遍。

第四步：边读边累加
形成累计值 acc：
acc = acc + hist[k]

第五步：做乘法和除法
计算：
(acc * sfreq) / freq

第六步：把结果写入 LUT[k]
于是 LUT 第 k 项就更新成当前环境下的细时间值。

第七步：处理下一个 bin
直到这一通道所有 LUT 项都更新完。

第八步：切到下一个通道
然后继续在线更新下一通道。
```
