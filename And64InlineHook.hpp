/*
 *  @date   : 2018/04/18
 *  @author : Rprop (r_prop@outlook.com)
 *  https://github.com/Rprop/And64InlineHook
 */
/*
 MIT License

 Copyright (c) 2018 Rprop (r_prop@outlook.com)

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */
#pragma once

/*
 * A64_MAX_BACKUPS: 定义最大可同时 Hook 的函数数量
 *
 * 每个被 Hook 的函数都需要一个独立的跳板(Trampoline)区域来保存原始指令的修复副本,
 * 这个值决定了预分配的跳板池大小。256 个槽位对于大多数应用场景已经足够。
 *
 * 跳板的作用: 当我们覆盖原函数入口时, 原函数开头的几条指令会被破坏。
 * 如果用户代码需要调用原函数(通过 result 参数返回的指针), 就需要先执行这些
 * 被保存并修复的指令, 然后跳转回原函数被覆盖部分之后继续执行。
 */
#define A64_MAX_BACKUPS 256

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * A64HookFunction - ARM64 内联 Hook 的主要接口
     *
     * @param symbol:  目标函数地址, 即要 Hook 的原函数入口点
     * @param replace: 替换函数地址, Hook 后所有对 symbol 的调用都会被重定向到这里
     * @param result:  输出参数, 返回跳板地址。通过这个指针可以调用原函数。
     *                 如果传入 NULL, 则不生成跳板(意味着无法调用原函数)
     *
     * 工作原理:
     * 该函数会修改 symbol 指向地址处的机器码, 将其替换为跳转到 replace 的指令。
     *
     * ARM64 架构下, 由于指令长度固定为 4 字节, 且 PC 相对跳转的范围有限(B 指令
     * 只能跳转 +/-128MB), 当目标地址超出范围时需要使用间接跳转:
     *   LDR X17, #8    ; 从 PC+8 处加载 64 位目标地址到 X17
     *   BR  X17        ; 通过 X17 寄存器间接跳转
     *   <64-bit addr>  ; 存放目标地址的 8 字节数据
     *
     * 这种方式需要覆盖 4-5 条指令(16-20字节), 而被覆盖的原指令会被复制到跳板
     * 并进行必要的修复(因为 PC 相对寻址的偏移量需要重新计算)。
     */
    void A64HookFunction(void *const symbol, void *const replace, void **result);

    /*
     * A64HookFunctionV - 带有自定义跳板缓冲区的 Hook 接口
     *
     * @param symbol:   目标函数地址
     * @param replace:  替换函数地址
     * @param rwx:      用户提供的可读/可写/可执行内存区域, 用作跳板
     * @param rwx_size: rwx 缓冲区的大小(字节)
     * @return:         成功返回跳板地址, 失败返回 NULL
     *
     * 与 A64HookFunction 的区别:
     * 此函数允许调用者自行管理跳板内存, 而不使用内置的跳板池。
     * 这在以下场景很有用:
     *   - 需要 Hook 超过 A64_MAX_BACKUPS 个函数
     *   - 需要精确控制跳板内存的位置(例如需要在特定地址范围内)
     *   - 需要在 Hook 之后释放跳板内存
     *
     * 注意: rwx 必须是具有执行权限的内存, 且大小至少为 count * 10 * sizeof(uint32_t)
     * 其中 count 是被覆盖的指令数量(通常为 4 或 5)。
     */
    void *A64HookFunctionV(void *const symbol, void *const replace,
                           void *const rwx, const uintptr_t rwx_size);

#ifdef __cplusplus
}
#endif