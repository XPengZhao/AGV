ALIENTEK STM32开发板SYSTEM文件夹，采用V3.5库的头文件，使得代码可以完美移植到库函数里面使用，不过注意，在新建工程的时候，请在：Options for Target-->C/C++ 选项卡里面 Preprocessor Symbols 栏定义你的STM32芯片容量。比如:ALIENTEK 战舰STM32开发板用户，使用的是大容量的STM32芯片，则在Define栏输入：STM32F10X_HD 

对于STM32F103系列芯片，设置原则如下：
16KB≤FLASH≤32KB       选择：STM32F10X_LD
64KB≤FLASH≤128KB      选择：STM32F10X_MD
256KB≤FLASH≤512KB     选择：STM32F10X_HD

