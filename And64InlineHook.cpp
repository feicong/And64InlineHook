/*
 * Copyright (c) 2025-2026 fei_cong(https://github.com/feicong/feicong-course)
 *
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
#define  __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h>
#include <android/log.h>

#if defined(__aarch64__)

#include "And64InlineHook.hpp"

/*
 * A64_MAX_INSTRUCTIONS: 指定 Hook 时需要覆盖(并修复)的最大原始指令数量
 *
 * 在 ARM64 架构中, 实现远距离跳转需要使用 LDR+BR 组合加上一个 64 位地址,
 * 共占用 16 字节(4条指令的空间)。由于 8 字节对齐的要求, 可能需要额外的 NOP,
 * 因此最多覆盖 5 条原始指令。
 *
 * 被覆盖的指令需要复制到跳板并进行修复, 这里设置上限为 5 条。
 */
#define   A64_MAX_INSTRUCTIONS 5

/*
 * A64_MAX_REFERENCES: 指令修复时的最大交叉引用数量
 *
 * 当被复制的指令之间存在相互跳转时(例如第3条指令跳转到第1条), 需要记录这些
 * 引用关系并在所有指令都被处理后进行修复。每条指令最多可能有 2 个待修复的引用。
 */
#define   A64_MAX_REFERENCES   (A64_MAX_INSTRUCTIONS * 2)

/*
 * A64_NOP: ARM64 的空操作指令编码
 *
 * NOP 指令(No Operation)不执行任何操作, 仅消耗一个 CPU 周期。
 * 在 Hook 中主要用于对齐目的: ARM64 的 LDR 指令从 PC 相对地址加载 64 位数据时,
 * 该数据必须 8 字节对齐。当目标地址未对齐时, 插入 NOP 来调整位置。
 *
 * 指令编码: 0xd503201f = 0b11010101000000110010000000011111
 * 属于 HINT 指令族, 具体为 HINT #0, 即 NOP。
 */
#define   A64_NOP              0xd503201fu

#define   A64_JNIEXPORT        __attribute__((visibility("default")))
#define   A64_LOGE(...)        ((void)__android_log_print(ANDROID_LOG_ERROR, "A64_HOOK", __VA_ARGS__))
#ifndef NDEBUG
# define  A64_LOGI(...)        ((void)__android_log_print(ANDROID_LOG_INFO, "A64_HOOK", __VA_ARGS__))
#else
# define  A64_LOGI(...)        ((void)0)
#endif // NDEBUG

/*
 * instruction 类型定义: 指向指令指针的指针
 *
 * 使用双重指针是为了让修复函数能够同时推进输入和输出位置。
 * __restrict 关键字告诉编译器这些指针不会互相别名, 便于优化。
 */
typedef uint32_t *__restrict *__restrict instruction;

/*
 * context 结构体: 指令修复的上下文信息
 *
 * 当原始指令被复制到跳板时, 涉及 PC 相对寻址的指令需要重新计算偏移量。
 * 更复杂的情况是: 被复制的多条指令之间可能存在相互跳转(例如循环),
 * 这时需要等所有指令都处理完毕才能确定最终的跳转目标地址。
 *
 * context 结构体就是用来跟踪这些待修复的引用关系。
 */
struct context
{
    /*
     * fix_info: 记录一个待修复的引用
     *
     * 当发现某条指令跳转到另一条尚未处理的指令时, 无法立即计算正确的偏移量。
     * 此时将该引用信息记录下来, 等目标指令被处理后再回填正确的偏移。
     */
    struct fix_info
    {
        uint32_t *bp;  // 需要被修复的指令地址(跳板中)
        uint32_t  ls;  // 偏移量字段在指令中的左移位数(用于将偏移放到正确的位域)
        uint32_t  ad;  // 偏移量字段的掩码(用于与运算提取或填充偏移量位域)
    };

    /*
     * insns_info: 单条指令的修复信息
     *
     * insu/ins/insp: 使用 union 存储指令在跳板中的地址, 可以按不同类型访问
     *   - insu: 作为无符号 64 位整数(用于地址比较)
     *   - ins:  作为有符号 64 位整数(用于偏移计算)
     *   - insp: 作为指针(用于内存操作)
     *
     * fmap: 该指令被其他指令引用的修复映射表
     */
    struct insns_info
    {
        union
        {
            uint64_t insu;  // 跳板中对应指令的地址(无符号)
            int64_t  ins;   // 跳板中对应指令的地址(有符号, 便于计算偏移)
            void    *insp;  // 跳板中对应指令的地址(指针形式)
        };
        fix_info fmap[A64_MAX_REFERENCES];  // 引用此指令的其他指令的修复信息
    };

    int64_t    basep;  // 原始指令序列的起始地址
    int64_t    endp;   // 原始指令序列的结束地址
    insns_info dat[A64_MAX_INSTRUCTIONS];  // 每条原始指令的修复信息

public:
    /*
     * is_in_fixing_range: 判断一个绝对地址是否位于正在修复的指令范围内
     *
     * 用途: 当处理一条跳转指令时, 如果跳转目标也在被覆盖的指令范围内,
     * 则需要特殊处理(因为目标指令也会被移动到跳板)。
     */
    inline bool is_in_fixing_range(const int64_t absolute_addr) {
        return absolute_addr >= this->basep && absolute_addr < this->endp;
    }

    /*
     * get_ref_ins_index: 根据绝对地址计算指令在数组中的索引
     *
     * ARM64 每条指令固定 4 字节, 因此索引 = (地址 - 基址) / 4
     */
    inline intptr_t get_ref_ins_index(const int64_t absolute_addr) {
        return static_cast<intptr_t>((absolute_addr - this->basep) / sizeof(uint32_t));
    }

    /*
     * get_and_set_current_index: 记录当前指令在跳板中的位置, 并返回其索引
     *
     * 在处理每条指令时调用, 将跳板中的输出位置记录到 dat 数组中。
     * 这样后续处理的指令如果需要跳转到此指令, 就能知道目标地址。
     */
    inline intptr_t get_and_set_current_index(uint32_t *__restrict inp, uint32_t *__restrict outp) {
        intptr_t current_idx = this->get_ref_ins_index(reinterpret_cast<int64_t>(inp));
        this->dat[current_idx].insp = outp;
        return current_idx;
    }

    /*
     * reset_current_ins: 更新指令在跳板中的位置
     *
     * 当因为对齐需要插入 NOP 时, 指令的实际位置会改变, 需要更新记录。
     */
    inline void reset_current_ins(const intptr_t idx, uint32_t *__restrict outp) {
        this->dat[idx].insp = outp;
    }

    /*
     * insert_fix_map: 插入一条待修复的引用记录
     *
     * 当指令 A 跳转到指令 B, 但 B 还未被处理时, 调用此函数记录:
     *   - bp: 指令 A 在跳板中的地址(需要被修复的位置)
     *   - ls: 偏移量字段在指令编码中的左移位数
     *   - ad: 偏移量字段的掩码
     *
     * 等指令 B 被处理后, process_fix_map 会使用这些信息回填正确的偏移。
     */
    void insert_fix_map(const intptr_t idx, uint32_t *bp, uint32_t ls = 0u, uint32_t ad = 0xffffffffu) {
        for (auto &f : this->dat[idx].fmap) {
            if (f.bp == NULL) {
                f.bp = bp;
                f.ls = ls;
                f.ad = ad;
                return;
            }
        }
        // 如果执行到这里说明 fmap 已满, 这是代码设计的缺陷
    }

    /*
     * process_fix_map: 处理所有引用当前指令的待修复项
     *
     * 当一条指令被处理完毕后调用此函数。遍历所有记录的引用, 计算正确的
     * PC 相对偏移量并回填到对应的指令中。
     *
     * 偏移计算公式: (目标地址 - 源指令地址) >> 2
     * ARM64 的 PC 相对偏移以 4 字节(即一条指令)为单位, 所以除以 4(右移 2 位)。
     *
     * 然后将偏移量左移 ls 位, 与 ad 掩码做与运算, 再或到原指令上。
     */
    void process_fix_map(const intptr_t idx) {
        for (auto &f : this->dat[idx].fmap) {
            if (f.bp == NULL) break;
            *(f.bp) = *(f.bp) | (((int32_t(this->dat[idx].ins - reinterpret_cast<int64_t>(f.bp)) >> 2) << f.ls) & f.ad);
            f.bp = NULL;
        }
    }
};

//-------------------------------------------------------------------------
// 辅助宏定义
//-------------------------------------------------------------------------

#define __intval(p)                reinterpret_cast<intptr_t>(p)
#define __uintval(p)               reinterpret_cast<uintptr_t>(p)
#define __ptr(p)                   reinterpret_cast<void *>(p)
#define __page_size                4096
#define __page_align(n)            __align_up(static_cast<uintptr_t>(n), __page_size)
#define __ptr_align(x)             __ptr(__align_down(reinterpret_cast<uintptr_t>(x), __page_size))
#define __align_up(x, n)           (((x) + ((n) - 1)) & ~((n) - 1))
#define __align_down(x, n)         ((x) & -(n))
#define __countof(x)               static_cast<intptr_t>(sizeof(x) / sizeof((x)[0])) // must be signed
#define __atomic_increase(p)       __sync_add_and_fetch(p, 1)
#define __sync_cmpswap(p, v, n)    __sync_bool_compare_and_swap(p, v, n)
#define __predict_true(exp)        __builtin_expect((exp) != 0, 1)

/*
 * __flush_cache: 刷新指令缓存
 *
 * ARM64 采用哈佛架构, 指令缓存(I-Cache)和数据缓存(D-Cache)是分离的。
 * 当我们通过数据操作修改了代码段的内容后, 修改首先只反映在数据缓存中,
 * 指令缓存中仍是旧的内容。如果不刷新指令缓存, CPU 可能继续执行旧指令。
 *
 * __builtin___clear_cache 是 GCC 内建函数, 用于确保:
 *   - 数据缓存中的修改被写回到主存
 *   - 指令缓存中对应范围的缓存行被无效化
 *   - 所有核心都能看到新的代码
 *
 * 这是 inline hook 正确工作的关键步骤, 必须在修改代码后调用。
 */
#define __flush_cache(c, n)        __builtin___clear_cache(reinterpret_cast<char *>(c), reinterpret_cast<char *>(c) + n)

/*
 * __make_rwx: 修改内存页的保护属性为可读/可写/可执行
 *
 * 代码段(.text)通常被映射为只读+可执行(R-X), 以防止意外或恶意修改。
 * 要进行 inline hook, 必须先将目标内存页修改为可写。
 *
 * 参数说明:
 *   - p: 要修改的地址
 *   - n: 要修改的长度
 *
 * mprotect 要求地址必须页对齐(4KB), 因此需要向下对齐到页边界。
 * 如果修改范围跨越页边界, 需要保护两个页。
 *
 * 返回值: 0 表示成功, -1 表示失败(errno 包含错误码)
 */
#define __make_rwx(p, n)           ::mprotect(__ptr_align(p), \
                                              __page_align(__uintval(p) + n) != __page_align(__uintval(p)) ? __page_align(n) + __page_size : __page_align(n), \
                                              PROT_READ | PROT_WRITE | PROT_EXEC)

//-------------------------------------------------------------------------

/*
 * __fix_branch_imm: 修复无条件分支指令 B 和 BL
 *
 * ARM64 的 B(无条件跳转) 和 BL(带链接的跳转, 即函数调用) 指令使用 26 位的
 * PC 相对偏移, 范围是 +/-128MB。当原始指令被复制到跳板后, 跳板与原目标地址
 * 的距离可能超出这个范围, 此时必须使用间接跳转来实现。
 *
 * 指令格式:
 *   B  imm26: 0b000101[imm26]           操作码 0x14000000
 *   BL imm26: 0b100101[imm26]           操作码 0x94000000
 *
 * 偏移量编码: imm26 是以 4 字节为单位的有符号偏移, 实际偏移 = imm26 << 2
 *
 * 修复策略:
 *   情况 1: 新偏移在 +/-128MB 范围内 -> 直接修改偏移量字段
 *   情况 2: 超出范围 -> 转换为间接跳转
 *
 * 对于 B 指令的间接跳转实现:
 *     LDR X17, #8      ; 从 PC+8 处加载 64 位目标地址到 X17
 *     BR X17           ; 通过 X17 寄存器跳转
 *     <64-bit target>  ; 目标地址(8字节, 必须 8 字节对齐)
 *
 *   使用 X17 寄存器是因为 ARM64 调用约定规定 X17 是临时寄存器(IP1),
 *   可以被过程链接表(PLT)代码使用, 不需要保存/恢复。
 *
 * 对于 BL 指令的间接跳转实现:
 *     LDR X17, #12     ; 从 PC+12 处加载目标地址(跳过 ADR 和 BR 指令)
 *     ADR X30, #16     ; 将返回地址(当前 PC+16)存入链接寄存器 X30
 *     BR X17           ; 跳转到目标函数
 *     <64-bit target>  ; 目标地址
 *
 *   BL 指令需要额外设置返回地址(X30/LR), 因为 BR 不像 BLR 那样自动设置 LR。
 *   ADR 指令计算返回地址: PC + 16 正好是数据之后的下一条指令位置。
 *
 * 关于 8 字节对齐的说明:
 *   LDR Xt, label 指令从 PC 相对地址加载 64 位数据, ARM64 要求该数据地址
 *   必须 8 字节对齐。如果当前位置不满足对齐要求, 需要插入 NOP 进行调整。
 *
 *   对于 B 指令(4 条指令版本): 数据在 outpp+2 处, 需要 (outpp+2) 8 字节对齐
 *     - 如果 outpp 是 8 对齐 -> outpp+2 是 8+8=16 字节对齐, 满足
 *     - 如果 outpp 是 4 对齐 -> outpp+2 是 4+8=12 字节对齐, 不满足, 需要插入 NOP
 *
 *   对于 BL 指令(5 条指令版本): 数据在 outpp+3 处, 需要 (outpp+3) 8 字节对齐
 *     - 如果 outpp 是 8 对齐 -> outpp+3 是 8+12=20 字节对齐, 不满足, 需要插入 NOP
 *     - 如果 outpp 是 4 对齐 -> outpp+3 是 4+12=16 字节对齐, 满足
 *
 *   这就是代码中 B 和 BL 对齐条件相反的原因。
 */
static bool __fix_branch_imm(instruction inpp, instruction outpp, context *ctxp)
{
    static constexpr uint32_t mbits = 6u;                 // 操作码占用的高位数量
    static constexpr uint32_t mask  = 0xfc000000u;        // 操作码掩码: 高 6 位 0b11111100000000000000000000000000
    static constexpr uint32_t rmask = 0x03ffffffu;        // 立即数掩码: 低 26 位 0b00000011111111111111111111111111
    static constexpr uint32_t op_b  = 0x14000000u;        // B 指令操作码 "b"  ADDR_PCREL26
    static constexpr uint32_t op_bl = 0x94000000u;        // BL 指令操作码 "bl" ADDR_PCREL26

    const uint32_t ins = *(*inpp);
    const uint32_t opc = ins & mask;
    switch (opc) {
    case op_b:
    case op_bl:
        {
            intptr_t current_idx  = ctxp->get_and_set_current_index(*inpp, *outpp);

            /*
             * 计算跳转的绝对目标地址
             *
             * imm26 是有符号数, 需要进行符号扩展:
             *   1. ins << mbits: 将 imm26 移到高位, 低位补 0
             *   2. 转为 int32_t 后右移: C 语言保证有符号数右移是算术右移(保留符号位)
             *   3. 右移 (mbits - 2) 位恢复原始偏移, 但保留了 <<2 的效果(因为偏移以 4 字节为单位)
             *
             * 最终 absolute_addr = 当前指令地址 + 有符号扩展后的字节偏移
             */
            int64_t absolute_addr = reinterpret_cast<int64_t>(*inpp) + (static_cast<int32_t>(ins << mbits) >> (mbits - 2u));

            /*
             * 计算从跳板中的新位置到目标的偏移
             * 右移 2 位是因为指令中存储的偏移以 4 字节为单位
             */
            int64_t new_pc_offset = static_cast<int64_t>(absolute_addr - reinterpret_cast<int64_t>(*outpp)) >> 2;

            // 检查目标是否也在被修复的范围内(即也被复制到了跳板)
            bool special_fix_type = ctxp->is_in_fixing_range(absolute_addr);

            /*
             * 判断是否需要转换为绝对跳转
             *
             * 如果目标不在修复范围内, 且新偏移超出 26 位有符号数的范围(+/-128MB),
             * 则必须使用 LDR+BR 的绝对跳转方式。
             *
             * rmask >> 1 = 0x01ffffff = 33554431, 约为 +/-128MB 的单位数
             */
            if (!special_fix_type && llabs(new_pc_offset) >= (rmask >> 1)) {
                // 检查数据位置的对齐情况: outpp+2 需要 8 字节对齐
                bool b_aligned = (reinterpret_cast<uint64_t>(*outpp + 2) & 7u) == 0u;

                if (opc == op_b) {
                    // B 指令: 数据在 outpp+2, 如果 outpp+2 不是 8 字节对齐则需要 NOP
                    if (b_aligned != true) {
                        (*outpp)[0] = A64_NOP;
                        ctxp->reset_current_ins(current_idx, ++(*outpp));
                    }
                    /*
                     * 生成 B 的间接跳转序列:
                     *   0x58000051 = LDR X17, #8   ; 0x58 表示 LDR (literal), #8 >> 2 = 2 存入 imm19
                     *   0xd61f0220 = BR X17        ; 0xd61f 是 BR 的操作码, 0x0220 编码 X17
                     */
                    (*outpp)[0] = 0x58000051u;      // LDR X17, #0x8
                    (*outpp)[1] = 0xd61f0220u;      // BR X17
                    memcpy(*outpp + 2, &absolute_addr, sizeof(absolute_addr));  // 8 字节目标地址
                    *outpp += 4;
                } else {
                    // BL 指令: 数据在 outpp+3, 对齐条件相反
                    if (b_aligned == true) {
                        (*outpp)[0] = A64_NOP;
                        ctxp->reset_current_ins(current_idx, ++(*outpp));
                    } //if
                    /*
                     * 生成 BL 的间接跳转序列:
                     *   0x58000071 = LDR X17, #12  ; 加载目标地址, 偏移 12 = 3 条指令
                     *   0x1000009e = ADR X30, #16  ; 计算返回地址 = PC + 16, 存入 LR
                     *   0xd61f0220 = BR X17        ; 跳转
                     *
                     * ADR 指令编码: 0x10000000 | (immlo << 29) | (immhi << 5) | Rd
                     *   immhi:immlo = 16 >> 2 = 4 = 0b100, immlo = 0, immhi = 0b10 = 2
                     *   ADR X30 = 0x10000000 | (0 << 29) | (2 << 5) | 30 = 0x1000009e
                     *   等等, 让我重新计算: imm = 16, imm 的低 2 位是 immlo, 高位是 immhi
                     *   16 = 0b10000, immlo = 0b00, immhi = 0b100 = 4
                     *   ADR = 0x10000000 | (immlo << 29) | (immhi << 5) | Rd
                     *       = 0x10000000 | (0 << 29) | (4 << 5) | 30
                     *       = 0x10000000 | 0 | 0x80 | 0x1e
                     *       = 0x1000009e
                     */
                    (*outpp)[0] = 0x58000071u;      // LDR X17, #12
                    (*outpp)[1] = 0x1000009eu;      // ADR X30, #16
                    (*outpp)[2] = 0xd61f0220u;      // BR X17
                    memcpy(*outpp + 3, &absolute_addr, sizeof(absolute_addr));
                    *outpp += 5;
                } //if
            } else {
                /*
                 * 新偏移在范围内, 或者目标也在修复范围内需要特殊处理
                 */
                if (special_fix_type) {
                    // 目标指令也被复制到了跳板
                    intptr_t ref_idx = ctxp->get_ref_ins_index(absolute_addr);
                    if (ref_idx <= current_idx) {
                        // 目标指令已经被处理过, 可以直接计算新偏移
                        new_pc_offset = static_cast<int64_t>(ctxp->dat[ref_idx].ins - reinterpret_cast<int64_t>(*outpp)) >> 2;
                    } else {
                        // 目标指令尚未处理, 记录待修复项, 稍后回填
                        ctxp->insert_fix_map(ref_idx, *outpp, 0u, rmask);
                        new_pc_offset = 0;  // 临时填 0, 之后会被修复
                    }
                }

                // 生成新指令: 保留操作码, 更新偏移量
                (*outpp)[0] = opc | (new_pc_offset & ~mask);
                ++(*outpp);
            } //if

            ++(*inpp);
            return ctxp->process_fix_map(current_idx), true;
        }
    }
    return false;
}

//-------------------------------------------------------------------------

/*
 * __fix_cond_comp_test_branch: 修复条件分支和比较测试分支指令
 *
 * 这类指令的跳转范围比 B/BL 更小, 需要特别处理:
 *
 *   B.cond label: 条件分支, 使用 19 位 PC 相对偏移, 范围 +/-1MB
 *     格式: 0b01010100[imm19]0[cond]   操作码掩码 0xff000010
 *
 *   CBZ/CBNZ Rt, label: 比较并分支(为零/非零), 19 位偏移, +/-1MB
 *     CBZ:  0b[sf]0110100[imm19][Rt]   操作码 0x34000000
 *     CBNZ: 0b[sf]0110101[imm19][Rt]   操作码 0x35000000
 *
 *   TBZ/TBNZ Rt, #bit, label: 测试位并分支, 14 位偏移, +/-32KB
 *     TBZ:  0b[b5]0110110[b40][imm14][Rt]   操作码 0x36000000
 *     TBNZ: 0b[b5]0110111[b40][imm14][Rt]   操作码 0x37000000
 *
 * 修复策略:
 *   如果新偏移在范围内, 直接修改偏移量字段。
 *   如果超出范围, 需要转换为条件跳转 + 绝对跳转的组合:
 *
 *     B.cond  #8         ; 条件满足时跳过下一条 B 指令
 *     B       #20        ; 条件不满足时跳过整个绝对跳转序列
 *     LDR     X17, #8    ; 加载目标地址
 *     BR      X17        ; 跳转
 *     <64-bit target>    ; 目标地址
 *
 *   这种转换的逻辑是: 将短距离条件跳转改为跳转到附近的绝对跳转代码,
 *   同时用一个短距离无条件跳转来跳过这段代码(当条件不满足时)。
 */
static bool __fix_cond_comp_test_branch(instruction inpp, instruction outpp, context *ctxp)
{
    // 偏移量字段在指令中的起始位置(从第 5 位开始)
    static constexpr uint32_t lsb     = 5u;

    // B.cond 和 CBZ/CBNZ 的掩码: 保留 Rt 字段(低 5 位)和操作码(高 8 位)
    static constexpr uint32_t lmask01 = 0xff00001fu; // 0b11111111000000000000000000011111

    // B.cond 的操作码掩码和值
    static constexpr uint32_t mask0   = 0xff000010u;      // 检测 B.cond: bit4 必须为 0 0b11111111000000000000000000010000
    static constexpr uint32_t op_bc   = 0x54000000u;      // B.cond 基础操作码 "b.c"  ADDR_PCREL19

    // CBZ/CBNZ 的操作码掩码和值(最高位 sf 不参与判断)
    static constexpr uint32_t mask1   = 0x7f000000u; // 0b01111111000000000000000000000000
    static constexpr uint32_t op_cbz  = 0x34000000u; // "cbz"  Rt, ADDR_PCREL19
    static constexpr uint32_t op_cbnz = 0x35000000u; // "cbnz" Rt, ADDR_PCREL19
    static constexpr uint32_t lmask2  = 0xfff8001fu; // 0b11111111111110000000000000011111
    static constexpr uint32_t mask2   = 0x7f000000u; // 0b01111111000000000000000000000000
    static constexpr uint32_t op_tbz  = 0x36000000u; // 0b00110110000000000000000000000000 "tbz"  Rt, BIT_NUM, ADDR_PCREL14
    static constexpr uint32_t op_tbnz = 0x37000000u; // 0b00110111000000000000000000000000 "tbnz" Rt, BIT_NUM, ADDR_PCREL14

    const uint32_t ins = *(*inpp);
    uint32_t     lmask = lmask01;

    // 依次检测是哪种指令
    if ((ins & mask0) != op_bc) {
        uint32_t opc = ins & mask1;
        if (opc != op_cbz && opc != op_cbnz) {
            opc = ins & mask2;
            if (opc != op_tbz && opc != op_tbnz) {
                return false;  // 不是我们要处理的指令类型
            }
            lmask = lmask2;  // TBZ/TBNZ 使用不同的掩码
        }
    }

    /*
     * 计算偏移量字段的最高有效位位置
     * __builtin_clz 计算前导零数量, ~lmask 的前导零数量就是掩码的最高有效位位置
     */
    const uint32_t msb    = __builtin_clz(~lmask);

    intptr_t current_idx  = ctxp->get_and_set_current_index(*inpp, *outpp);

    /*
     * 提取并符号扩展偏移量, 计算绝对目标地址
     *
     * 1. ins & ~lmask: 提取偏移量字段(清除操作码和寄存器字段)
     * 2. << msb: 将偏移量移到最高位进行符号扩展
     * 3. 转为 int32_t 后 >> (lsb - 2 + msb): 算术右移恢复偏移量并保留 <<2 效果
     */
    int64_t absolute_addr = reinterpret_cast<int64_t>(*inpp) + (static_cast<int32_t>((ins & ~lmask) << msb) >> (lsb - 2u + msb));
    int64_t new_pc_offset = static_cast<int64_t>(absolute_addr - reinterpret_cast<int64_t>(*outpp)) >> 2; // shifted
    bool special_fix_type = ctxp->is_in_fixing_range(absolute_addr);

    /*
     * 检查是否需要转换为绝对跳转
     * ~lmask >> (lsb + 1) 计算偏移量字段能表示的最大正值
     */
    if (!special_fix_type && llabs(new_pc_offset) >= (~lmask >> (lsb + 1))) {
        // 数据在 outpp+4, 需要 8 字节对齐
        if ((reinterpret_cast<uint64_t>(*outpp + 4) & 7u) != 0u) {
            (*outpp)[0] = A64_NOP;
            ctxp->reset_current_ins(current_idx, ++(*outpp));
        } //if

        /*
         * 生成条件跳转 + 绝对跳转序列:
         *
         * outpp[0]: 原条件分支指令, 但目标改为 #8(即跳过下一条 B 指令)
         *           (8 >> 2) << lsb 计算出要填入的偏移量字段值
         *
         * outpp[1]: B #20 (= 5 条指令 = 20 字节), 跳过整个绝对跳转序列
         *           0x14000005 = 0x14000000 | 5
         *
         * outpp[2-3]: LDR X17, #8 + BR X17 绝对跳转
         * outpp[4-5]: 64 位目标地址
         */
        (*outpp)[0] = (((8u >> 2u) << lsb) & ~lmask) | (ins & lmask);  // 修改后的条件分支 B.C #0x8
        (*outpp)[1] = 0x14000005u;      // B #0x14, 跳过绝对跳转序列
        (*outpp)[2] = 0x58000051u;      // LDR X17, #0x8
        (*outpp)[3] = 0xd61f0220u;      // BR X17
        memcpy(*outpp + 4, &absolute_addr, sizeof(absolute_addr));
        *outpp += 6;
    } else {
        // 新偏移在范围内
        if (special_fix_type) {
            intptr_t ref_idx = ctxp->get_ref_ins_index(absolute_addr);
            if (ref_idx <= current_idx) {
                new_pc_offset = static_cast<int64_t>(ctxp->dat[ref_idx].ins - reinterpret_cast<int64_t>(*outpp)) >> 2;
            } else {
                ctxp->insert_fix_map(ref_idx, *outpp, lsb, ~lmask);
                new_pc_offset = 0;
            }
        }

        // 生成新指令: 将新偏移量编码到指令中
        (*outpp)[0] = (static_cast<uint32_t>(new_pc_offset << lsb) & ~lmask) | (ins & lmask);
        ++(*outpp);
    } //if

    ++(*inpp);
    return ctxp->process_fix_map(current_idx), true;
}

//-------------------------------------------------------------------------

/*
 * __fix_loadlit: 修复 PC 相对字面量加载指令
 *
 * 这类指令从 PC 相对地址加载常量数据到寄存器, 常用于:
 *   - 加载浮点常量
 *   - 加载超出立即数范围的整数常量
 *   - 加载地址常量
 *
 * 支持的指令:
 *   LDR Wt/Xt, label:   加载 32/64 位整数, 19 位偏移, +/-1MB
 *   LDR St/Dt/Qt, label: 加载 32/64/128 位浮点数, 19 位偏移, +/-1MB
 *   LDRSW Xt, label:     加载 32 位有符号整数并符号扩展到 64 位
 *   PRFM label:          预取内存(不实际加载数据, 只是给缓存提示)
 *
 * 指令编码:
 *   LDR Wt:   0b00011000[imm19][Rt]     opc=00, size=1x -> 32位
 *   LDR Xt:   0b01011000[imm19][Rt]     opc=01, size=1x -> 64位
 *   LDR St:   0b00011100[imm19][Rt]     opc=00, V=1 -> 32位浮点
 *   LDR Dt:   0b01011100[imm19][Rt]     opc=01, V=1 -> 64位浮点
 *   LDR Qt:   0b10011100[imm19][Rt]     opc=10, V=1 -> 128位浮点
 *   LDRSW Xt: 0b10011000[imm19][Rt]     有符号字加载
 *   PRFM:     0b11011000[imm19][Rt]     内存预取
 *
 * 对齐要求:
 *   - 32 位加载: 数据地址必须 4 字节对齐
 *   - 64 位加载: 数据地址必须 8 字节对齐
 *   - 128 位加载: 数据地址必须 16 字节对齐
 *
 * 修复策略:
 *   如果新偏移在范围内且满足对齐要求, 直接修改偏移量。
 *   如果超出范围或对齐不满足, 将数据内联到跳板中:
 *
 *     LDR Xt, #8         ; 从后面的内联数据加载
 *     B #offset          ; 跳过内联数据
 *     <data>             ; 内联的常量数据
 *
 *   这种方式直接把原来的数据复制到跳板中, 避免了距离问题。
 */
static bool __fix_loadlit(instruction inpp, instruction outpp, context *ctxp)
{
    const uint32_t ins = *(*inpp);

    /*
     * PRFM (预取) 指令特殊处理: 直接跳过
     *
     * PRFM 是性能提示指令, 不影响程序正确性。在跳板中执行预取原地址
     * 的数据通常没有意义(因为执行上下文已经改变), 所以直接忽略。
     *
     * PRFM literal 编码: 0b11011000[imm19][Rt]
     */
    if ((ins & 0xff000000u) == 0xd8000000u) {
        ctxp->process_fix_map(ctxp->get_and_set_current_index(*inpp, *outpp));
        ++(*inpp);
        return true;
    }

    static constexpr uint32_t msb        = 8u;             // 高 8 位是操作码
    static constexpr uint32_t lsb        = 5u;             // 低 5 位是寄存器
    static constexpr uint32_t mask_30    = 0x40000000u;    // bit30: opc[0], 区分 32/64 位
    static constexpr uint32_t mask_31    = 0x80000000u;    // bit31: opc[1], 用于浮点 128 位检测
    static constexpr uint32_t lmask      = 0xff00001fu;    // 保留操作码和寄存器字段的掩码

    // LDR Wt/Xt 的掩码和操作码(bit26=V=0 表示通用寄存器)
    static constexpr uint32_t mask_ldr   = 0xbf000000u;
    static constexpr uint32_t op_ldr     = 0x18000000u;

    // LDR St/Dt/Qt 的掩码和操作码(bit26=V=1 表示 SIMD/FP 寄存器)
    static constexpr uint32_t mask_ldrv  = 0x3f000000u;
    static constexpr uint32_t op_ldrv    = 0x1c000000u;

    // LDRSW 的掩码和操作码
    static constexpr uint32_t mask_ldrsw = 0xff000000u;
    static constexpr uint32_t op_ldrsw   = 0x98000000u;

    uint32_t  mask     = mask_ldr;
    /*
     * 确定数据对齐要求
     *
     * faligned 表示对齐掩码: 数据地址 & faligned 必须为 0
     *   - 3 (0b011): 4 字节对齐, 用于 32 位加载
     *   - 7 (0b111): 8 字节对齐, 用于 64 位加载
     *   - 15 (0b1111): 16 字节对齐, 用于 128 位加载
     *
     * 对于 LDR Wt/Xt: bit30=0 为 32 位, bit30=1 为 64 位
     */
    uintptr_t faligned = (ins & mask_30) ? 7u : 3u;

    if ((ins & mask_ldr) != op_ldr) {
        // 不是通用寄存器加载, 检查是否是浮点寄存器加载
        mask = mask_ldrv;
        /*
         * 浮点加载的对齐要求:
         *   bit30=0, bit31=0: 32 位 (St) -> 4 字节对齐
         *   bit30=1, bit31=0: 64 位 (Dt) -> 8 字节对齐
         *   bit30=0, bit31=1: 128 位 (Qt) -> 16 字节对齐
         *   bit30=1, bit31=1: 保留(无效)
         */
        if (faligned != 7u)
            faligned = (ins & mask_31) ? 15u : 3u;

        if ((ins & mask_ldrv) != op_ldrv) {
            // 也不是浮点加载, 检查是否是 LDRSW
            if ((ins & mask_ldrsw) != op_ldrsw) {
                return false;  // 不是我们要处理的指令类型
            }
            mask     = mask_ldrsw;
            faligned = 7u;  // LDRSW 加载 32 位数据但扩展到 64 位寄存器
        }
    }

    intptr_t current_idx  = ctxp->get_and_set_current_index(*inpp, *outpp);

    /*
     * 计算数据的绝对地址
     *
     * LDR literal 的偏移量是以 4 字节为单位的, 且目标地址必须 4 字节对齐。
     * (ins << msb) >> (msb + lsb - 2): 符号扩展并乘以 4
     * & ~3: 确保结果 4 字节对齐(实际上由于 -2 已经是 4 的倍数)
     */
    int64_t absolute_addr = reinterpret_cast<int64_t>(*inpp) + ((static_cast<int32_t>(ins << msb) >> (msb + lsb - 2u)) & ~3u);

    int64_t new_pc_offset = static_cast<int64_t>(absolute_addr - reinterpret_cast<int64_t>(*outpp)) >> 2;
    bool special_fix_type = ctxp->is_in_fixing_range(absolute_addr);

    /*
     * 判断是否需要将数据内联到跳板
     *
     * 两种情况需要内联:
     * 1. special_fix_type: 数据在被修复的代码范围内(可能是混合代码/数据)
     * 2. 新偏移超出范围或对齐不满足
     *
     * 对齐检查: new_pc_offset + (faligned + 1 - 4) / 4 考虑了可能需要的 NOP 数量
     */
    if (special_fix_type || (llabs(new_pc_offset) + (faligned + 1u - 4u) / 4u) >= (~lmask >> (lsb + 1))) {
        // 插入 NOP 直到数据位置满足对齐要求
        while ((reinterpret_cast<uint64_t>(*outpp + 2) & faligned) != 0u) {
            *(*outpp)++ = A64_NOP;
        }
        ctxp->reset_current_ins(current_idx, *outpp);

        /*
         * 生成内联数据序列:
         *   LDR Xt, #8        ; 从 PC+8 处加载数据
         *   B #(8+ns*4)       ; 跳过内联数据
         *   <data>            ; 内联数据(大小由 faligned+1 决定)
         *
         * 注意: 如果原始数据所在内存是可写的(非常量), 这里复制的是一个快照。
         * 如果原程序后续修改了该数据, 跳板中的副本不会同步更新。
         * 这是一个已知限制, 对于常量数据没有问题。
         */
        uint32_t ns = static_cast<uint32_t>((faligned + 1) / sizeof(uint32_t));  // 数据占用的指令槽数
        (*outpp)[0] = (((8u >> 2u) << lsb) & ~mask) | (ins & lmask); // LDR #0x8
        (*outpp)[1] = 0x14000001u + ns;  // B #(4 + ns*4), 跳过数据, B #0xc
        memcpy(*outpp + 2, reinterpret_cast<void *>(absolute_addr), faligned + 1);
        *outpp += 2 + ns;
    } else {
        /*
         * 新偏移在范围内, 但需要确保对齐
         *
         * faligned >> 2: 将字节对齐要求转换为指令对齐要求
         *   - 4 字节对齐(faligned=3): faligned>>2 = 0, 无额外要求
         *   - 8 字节对齐(faligned=7): faligned>>2 = 1, 要求偏移是偶数
         *   - 16 字节对齐(faligned=15): faligned>>2 = 3, 要求偏移是 4 的倍数
         */
        faligned >>= 2;
        while ((new_pc_offset & faligned) != 0) {
            *(*outpp)++   = A64_NOP;
            new_pc_offset = static_cast<int64_t>(absolute_addr - reinterpret_cast<int64_t>(*outpp)) >> 2;
        }
        ctxp->reset_current_ins(current_idx, *outpp);

        // 生成新指令: 更新偏移量
        (*outpp)[0] = (static_cast<uint32_t>(new_pc_offset << lsb) & ~mask) | (ins & lmask);
        ++(*outpp);
    }

    ++(*inpp);
    return ctxp->process_fix_map(current_idx), true;
}

//-------------------------------------------------------------------------

/*
 * __fix_pcreladdr: 修复 PC 相对地址计算指令 ADR 和 ADRP
 *
 * 这两条指令用于将 PC 相对地址加载到寄存器, 是实现位置无关代码(PIC)的基础:
 *
 *   ADR Xd, label:  将 PC + offset 计算并存入 Xd, 21 位有符号偏移, 范围 +/-1MB
 *   ADRP Xd, label: 将 (PC & ~0xFFF) + (offset << 12) 存入 Xd, 用于计算页地址
 *
 * 编码格式:
 *   ADR:  0b0[immlo][10000][immhi][Rd]   操作码掩码 0x9f000000, 值 0x10000000
 *   ADRP: 0b1[immlo][10000][immhi][Rd]   操作码掩码 0x9f000000, 值 0x90000000
 *
 *   immlo: bit29-30, 2 位, 存储偏移量的低 2 位
 *   immhi: bit5-23, 19 位, 存储偏移量的高 19 位
 *   完整偏移量 = immhi:immlo (21 位有符号数)
 *   对于 ADRP, 实际偏移 = (immhi:immlo) << 12, 范围 +/-4GB
 *
 * 修复策略:
 *   ADR: 如果新偏移在范围内直接修改; 否则转为 LDR 加载绝对地址
 *   ADRP: 由于计算的是页地址, 很难正确修复(目标页可能改变),
 *         当前实现选择直接计算出绝对地址并用 LDR 加载
 *
 * 对于 ADR 超出范围的修复:
 *     LDR Xd, #8        ; 从后面加载预计算的绝对地址
 *     B #12             ; 跳过数据
 *     <64-bit addr>     ; 预计算的地址值
 *
 * 注意: 原 ADR/ADRP 指令计算的是地址, 不是加载数据。修复后我们直接把
 * 计算结果(地址值)存储在跳板中, 运行时加载这个预计算的值, 效果等价。
 */
static bool __fix_pcreladdr(instruction inpp, instruction outpp, context *ctxp)
{
    // Load a PC-relative address into a register
    // http://infocenter.arm.com/help/topic/com.arm.doc.100069_0608_00_en/pge1427897645644.html
    static constexpr uint32_t msb     = 8u;             // immhi 之上的高位数
    static constexpr uint32_t lsb     = 5u;             // immhi 的起始位
    static constexpr uint32_t mask    = 0x9f000000u;    // 操作码掩码(排除 immlo), 0b10011111000000000000000000000000
    static constexpr uint32_t rmask   = 0x0000001fu;    // Rd 字段掩码, 0b00000000000000000000000000011111
    static constexpr uint32_t lmask   = 0xff00001fu;    // 操作码 + Rd 掩码, 0b11111111000000000000000000011111
    static constexpr uint32_t fmask   = 0x00ffffffu;    // immhi:immlo 合并后的掩码(用于编码), 0b00000000111111111111111111111111
    static constexpr uint32_t max_val = 0x001fffffu;    // 21 位有符号数的最大正值, 0b00000000000111111111111111111111
    static constexpr uint32_t op_adr  = 0x10000000u;    // ADR 操作码, "adr"  Rd, ADDR_PCREL21
    static constexpr uint32_t op_adrp = 0x90000000u;    // ADRP 操作码, "adrp" Rd, ADDR_ADRP

    const uint32_t ins = *(*inpp);
    intptr_t current_idx;

    switch (ins & mask) {
    case op_adr:
        {
            current_idx = ctxp->get_and_set_current_index(*inpp, *outpp);

            /*
             * 提取并计算 ADR 的目标地址
             *
             * immlo 在 bit29-30: (ins << 1) >> 30 提取这 2 位
             * immhi 在 bit5-23: (ins << msb) >> (msb + lsb - 2) 提取并符号扩展
             * & ~3 清除低 2 位后与 immlo 合并得到完整偏移
             */
            int64_t lsb_bytes     = static_cast<uint32_t>(ins << 1u) >> 30u;
            int64_t absolute_addr = reinterpret_cast<int64_t>(*inpp) + (((static_cast<int32_t>(ins << msb) >> (msb + lsb - 2u)) & ~3u) | lsb_bytes);

            // ADR 的偏移是字节级别的, 不需要右移
            int64_t new_pc_offset = static_cast<int64_t>(absolute_addr - reinterpret_cast<int64_t>(*outpp));
            bool special_fix_type = ctxp->is_in_fixing_range(absolute_addr);

            if (!special_fix_type && llabs(new_pc_offset) >= (max_val >> 1)) {
                // 超出 ADR 范围, 转为 LDR 绝对地址
                if ((reinterpret_cast<uint64_t>(*outpp + 2) & 7u) != 0u) {
                    (*outpp)[0] = A64_NOP;
                    ctxp->reset_current_ins(current_idx, ++(*outpp));
                }

                /*
                 * 生成 LDR 替代序列:
                 *   0x58000000 | (2 << 5) | Rd = LDR Xd, #8
                 *   其中 (8 >> 2) << 5 = 2 << 5 = 0x40 作为 imm19 的值
                 *
                 *   0x14000003 = B #12, 跳过 8 字节数据
                 */
                (*outpp)[0] = 0x58000000u | (((8u >> 2u) << lsb) & ~mask) | (ins & rmask); // LDR Xd, #0x8
                (*outpp)[1] = 0x14000003u; // B #0xc
                memcpy(*outpp + 2, &absolute_addr, sizeof(absolute_addr));
                *outpp += 4;
            } else {
                // 新偏移在范围内
                if (special_fix_type) {
                    intptr_t ref_idx = ctxp->get_ref_ins_index(absolute_addr & ~3ull);
                    if (ref_idx <= current_idx) {
                        new_pc_offset = static_cast<int64_t>(ctxp->dat[ref_idx].ins - reinterpret_cast<int64_t>(*outpp));
                    } else {
                        ctxp->insert_fix_map(ref_idx, *outpp, lsb, fmask);
                        new_pc_offset = 0;
                    }
                }

                /*
                 * 编码新的 ADR 指令
                 *
                 * new_pc_offset << (lsb - 2): 将偏移的高位放到 immhi 位置
                 *   - new_pc_offset 是字节偏移
                 *   - 低 2 位需要放到 immlo (bit29-30), 高位放到 immhi (bit5-23)
                 *   - 这里 (lsb - 2) = 3, 左移 3 位相当于把 bit2-20 移到 bit5-23
                 *
                 * & fmask: 只保留 immhi:immlo 位域
                 * lmask 保留了原指令的 immlo 位(但实际上这里重新计算了整个偏移)
                 */
                (*outpp)[0] = (static_cast<uint32_t>(new_pc_offset << (lsb - 2u)) & fmask) | (ins & lmask);
                ++(*outpp);
            } //if
        }
        break;

    case op_adrp:
        {
            current_idx = ctxp->get_and_set_current_index(*inpp, *outpp);

            /*
             * 计算 ADRP 的目标页地址
             *
             * ADRP 计算公式: (PC & ~0xFFF) + SignExtend(imm, 21) << 12
             * 即: 当前 PC 所在页的基址 + 偏移量 * 4KB
             *
             * PC & ~0xfff: 清除低 12 位得到页基址
             * imm << 12: 偏移量左移 12 位(乘以 4KB)
             */
            int32_t lsb_bytes     = static_cast<uint32_t>(ins << 1u) >> 30u;
            int64_t absolute_addr = (reinterpret_cast<int64_t>(*inpp) & ~0xfffll) + ((((static_cast<int32_t>(ins << msb) >> (msb + lsb - 2u)) & ~3u) | lsb_bytes) << 12);

            A64_LOGI("ins = 0x%.8X, pc = %p, abs_addr = %p",
                     ins, *inpp, reinterpret_cast<int64_t *>(absolute_addr));

            if (ctxp->is_in_fixing_range(absolute_addr)) {
                /*
                 * ADRP 目标在修复范围内的特殊情况
                 *
                 * ADRP 计算的是页地址(低 12 位被清零), 如果原始代码使用 ADRP + ADD/LDR
                 * 组合来访问某个地址, 移动到跳板后这种组合可能失效。
                 *
                 * 理论上需要重新计算 ADRP 的偏移, 使得计算出的页地址仍然正确。
                 * 但这很复杂, 因为跳板的页地址可能与原地址的页地址不同。
                 *
                 * 当前实现选择直接保留原指令(这通常是错误的), 并记录日志。
                 * 实际使用中这种情况很少见, 因为很少有函数开头的几条指令中
                 * 既有 ADRP 又有该 ADRP 引用的数据。
                 */
                intptr_t ref_idx = ctxp->get_ref_ins_index(absolute_addr/* & ~3ull*/);
                if (ref_idx > current_idx) {
                    A64_LOGE("ref_idx must be less than or equal to current_idx!");
                }
                A64_LOGI("What is the correct way to fix this?");
                *(*outpp)++ = ins; // 直接复制原指令(可能不正确?),0x90000000u;
            } else {
                /*
                 * ADRP 目标不在修复范围内, 转为 LDR 绝对地址
                 *
                 * 由于 ADRP 计算的是页地址(可能与跳板不在同一页),
                 * 最安全的方式是预计算出绝对地址并用 LDR 加载。
                 */
                if ((reinterpret_cast<uint64_t>(*outpp + 2) & 7u) != 0u) {
                    (*outpp)[0] = A64_NOP;
                    ctxp->reset_current_ins(current_idx, ++(*outpp));
                }

                (*outpp)[0] = 0x58000000u | (((8u >> 2u) << lsb) & ~mask) | (ins & rmask);  // LDR Xd, #0x8
                (*outpp)[1] = 0x14000003u; // B #0xc
                memcpy(*outpp + 2, &absolute_addr, sizeof(absolute_addr));
                *outpp += 4;
            }
        }
        break;

    default:
        return false;
    }

    ctxp->process_fix_map(current_idx);
    ++(*inpp);
    return true;
}

//-------------------------------------------------------------------------

/*
 * __fix_instructions: 批量修复被覆盖的原始指令, 生成跳板代码
 *
 * 这是指令修复的主入口函数。它遍历每条原始指令, 判断其类型并调用相应的
 * 修复函数。不涉及 PC 相对寻址的指令可以直接复制。
 *
 * 修复完所有原始指令后, 还需要添加一条跳转回原函数的指令。这样当跳板代码
 * 执行完被覆盖的指令后, 会自动跳转回原函数继续执行剩余部分。
 *
 * 跳板代码结构:
 *   [修复后的原始指令 1]
 *   [修复后的原始指令 2]
 *   ...
 *   [修复后的原始指令 N]
 *   [跳转回原函数的指令]  ; 目标是原函数第 N+1 条指令的位置
 *
 * @param inp:   原始指令的起始地址
 * @param count: 需要修复的指令数量
 * @param outp:  跳板的起始地址(输出位置)
 */
static void __fix_instructions(uint32_t *__restrict inp, int32_t count, uint32_t *__restrict outp)
{
    context ctx;
    ctx.basep = reinterpret_cast<int64_t>(inp);
    ctx.endp  = reinterpret_cast<int64_t>(inp + count);
    memset(ctx.dat, 0, sizeof(ctx.dat));

    static_assert(sizeof(ctx.dat) / sizeof(ctx.dat[0]) == A64_MAX_INSTRUCTIONS,
                  "please use A64_MAX_INSTRUCTIONS!");
#ifndef NDEBUG
    if (count > A64_MAX_INSTRUCTIONS) {
        A64_LOGE("too many fixing instructions!");
    }
#endif // NDEBUG

    uint32_t *const outp_base = outp;

    /*
     * 逐条处理原始指令
     *
     * 每条指令依次尝试用各个修复函数处理。如果某个函数返回 true, 说明该指令
     * 已被处理(可能生成了一条或多条跳板指令), 继续下一条。
     *
     * 如果所有修复函数都返回 false, 说明该指令不涉及 PC 相对寻址, 直接复制即可。
     * 这类指令包括: 寄存器操作、立即数操作、内存间接寻址等。
     */
    while (--count >= 0) {
        if (__fix_branch_imm(&inp, &outp, &ctx)) continue;
        if (__fix_cond_comp_test_branch(&inp, &outp, &ctx)) continue;
        if (__fix_loadlit(&inp, &outp, &ctx)) continue;
        if (__fix_pcreladdr(&inp, &outp, &ctx)) continue;

        // 不涉及 PC 相对寻址的指令, 直接复制
        ctx.process_fix_map(ctx.get_and_set_current_index(inp, outp));
        *(outp++) = *(inp++);
    }

    /*
     * 生成跳转回原函数的指令
     *
     * 此时 inp 指向原函数中第一条未被覆盖的指令(即回调点)。
     * 需要从跳板的当前位置跳转到这个回调点。
     *
     * 与 __fix_branch_imm 类似, 如果距离在 +/-128MB 内可以用 B 指令,
     * 否则需要使用 LDR+BR 间接跳转。
     */
    static constexpr uint_fast64_t mask = 0x03ffffffu;  // B 指令的 26 位偏移掩码,0b00000011111111111111111111111111
    auto callback  = reinterpret_cast<int64_t>(inp);    // 回调点地址
    auto pc_offset = static_cast<int64_t>(callback - reinterpret_cast<int64_t>(outp)) >> 2;

    if (llabs(pc_offset) >= (mask >> 1)) {
        // 超出 B 指令范围, 使用 LDR+BR
        if ((reinterpret_cast<uint64_t>(outp + 2) & 7u) != 0u) {
            outp[0] = A64_NOP;
            ++outp;
        } //if
        outp[0] = 0x58000051u; // LDR X17, #0x8
        outp[1] = 0xd61f0220u; // BR X17
        *reinterpret_cast<int64_t *>(outp + 2) = callback;
        outp += 4;
    } else {
        // 在 B 指令范围内, 直接使用 B 指令
        outp[0] = 0x14000000u | (pc_offset & mask);  // B #offset， "B" ADDR_PCREL26
        ++outp;
    }

    // 刷新指令缓存, 确保 CPU 能执行新生成的跳板代码
    const uintptr_t total = (outp - outp_base) * sizeof(uint32_t);
    __flush_cache(outp_base, total); // necessary
}

//-------------------------------------------------------------------------

extern "C" {
    /*
     * __insns_pool: 预分配的跳板内存池
     *
     * 这是一个静态分配的数组, 用于存储所有 Hook 的跳板代码。
     * 数组按页对齐(__page_size = 4096), 便于 mprotect 修改权限。
     *
     * 每个跳板槽位大小: A64_MAX_INSTRUCTIONS * 10 * sizeof(uint32_t)
     *   = 5 * 10 * 4 = 200 字节
     *
     * 这个大小足够容纳:
     *   - 最多 5 条原始指令的修复版本(每条最多展开为多条)
     *   - 跳转回原函数的指令
     *
     * 总内存: 256 * 200 = 51200 字节 = 50KB
     */
    static __attribute__((__aligned__(__page_size))) uint32_t __insns_pool[A64_MAX_BACKUPS][A64_MAX_INSTRUCTIONS * 10];

    //-------------------------------------------------------------------------

    /*
     * A64HookInit: 初始化类, 用于在程序启动时设置跳板池的内存权限
     *
     * 使用静态对象的构造函数实现自动初始化。当包含此代码的共享库被加载时,
     * 静态对象 __init 会被构造, 其构造函数会被自动调用。
     *
     * 构造函数中调用 __make_rwx 将跳板池设置为可读/可写/可执行,
     * 这是后续写入和执行跳板代码的前提条件。
     */
    class A64HookInit
    {
    public:
        A64HookInit()
        {
            __make_rwx(__insns_pool, sizeof(__insns_pool));
            A64_LOGI("insns pool initialized.");
        }
    };
    static A64HookInit __init;

    //-------------------------------------------------------------------------

    /*
     * FastAllocateTrampoline: 从跳板池中快速分配一个槽位
     *
     * 使用原子操作实现线程安全的槽位分配, 无需加锁。
     * __atomic_increase 是一个原子递增操作, 多线程同时调用也能保证
     * 每个线程获得不同的索引值。
     *
     * @return: 成功返回跳板槽位的地址, 失败(池已满)返回 NULL
     */
    static uint32_t *FastAllocateTrampoline()
    {
        static_assert((A64_MAX_INSTRUCTIONS * 10 * sizeof(uint32_t)) % 8 == 0, "8-byte align");
        static volatile int32_t __index = -1;  // 初始为 -1, 第一次调用后变为 0

        int32_t i = __atomic_increase(&__index);
        if (__predict_true(i >= 0 && i < __countof(__insns_pool))) {
            return __insns_pool[i];
        }

        A64_LOGE("failed to allocate trampoline!");
        return NULL;
    }

    //-------------------------------------------------------------------------

    /*
     * A64HookFunctionV: 带自定义跳板缓冲区的 Hook 实现
     *
     * 这是 Hook 的核心实现函数。它完成以下工作:
     *   1. 备份并修复原函数开头的指令到跳板
     *   2. 将原函数入口替换为跳转到替换函数的代码
     *
     * Hook 后的执行流程:
     *   调用原函数 -> 跳转到替换函数 -> (可选)调用跳板执行原逻辑 -> 返回
     *
     * @param symbol:   要 Hook 的目标函数地址
     * @param replace:  替换函数地址
     * @param rwx:      跳板缓冲区(需要有 RWX 权限)
     * @param rwx_size: 跳板缓冲区大小
     * @return:         成功返回跳板地址, 失败返回 NULL
     */
    A64_JNIEXPORT void *A64HookFunctionV(void *const symbol, void *const replace,
                                         void *const rwx, const uintptr_t rwx_size)
    {
        static constexpr uint_fast64_t mask = 0x03ffffffu;  // B 指令偏移掩码 0b00000011111111111111111111111111

        uint32_t *trampoline = static_cast<uint32_t *>(rwx);
        uint32_t *original = static_cast<uint32_t *>(symbol);

        static_assert(A64_MAX_INSTRUCTIONS >= 5, "please fix A64_MAX_INSTRUCTIONS!");

        /*
         * 计算从原函数到替换函数的 PC 相对偏移
         *
         * 如果偏移在 +/-128MB 范围内, 只需覆盖一条指令(B 指令)。
         * 否则需要使用 LDR+BR 间接跳转, 需要覆盖 4-5 条指令。
         */
        auto pc_offset = static_cast<int64_t>(__intval(replace) - __intval(symbol)) >> 2;

        if (llabs(pc_offset) >= (mask >> 1)) {
            /*
             * 远距离跳转: 需要使用 LDR+BR 组合
             *
             * 指令序列:
             *   [NOP]             ; 可选, 用于对齐
             *   LDR X17, #8       ; 从 PC+8 加载目标地址
             *   BR  X17           ; 跳转到目标
             *   <64-bit address>  ; 替换函数地址
             *
             * 是否需要 NOP 取决于 original+2 的对齐情况:
             *   - 如果 (original+2) 已经 8 字节对齐 -> 不需要 NOP, 共 4 条指令
             *   - 否则 -> 需要 NOP 对齐, 共 5 条指令
             */
            int32_t count = (reinterpret_cast<uint64_t>(original + 2) & 7u) != 0u ? 5 : 4;

            if (trampoline) {
                // 检查跳板缓冲区大小
                if (rwx_size < count * 10u) {
                    A64_LOGE("rwx size is too small to hold %u bytes backup instructions!", count * 10u);
                    return NULL;
                }
                // 备份并修复原始指令
                __fix_instructions(original, count, trampoline);
            }

            // 修改原函数入口
            if (__make_rwx(original, 5 * sizeof(uint32_t)) == 0) {
                if (count == 5) {
                    // 需要 NOP 对齐
                    original[0] = A64_NOP;
                    ++original;
                }
                original[0] = 0x58000051u; // LDR X17, #0x8
                original[1] = 0xd61f0220u; // BR X17
                *reinterpret_cast<int64_t *>(original + 2) = __intval(replace);
                __flush_cache(symbol, 5 * sizeof(uint32_t));

                A64_LOGI("inline hook %p->%p successfully! %zu bytes overwritten",
                         symbol, replace, 5 * sizeof(uint32_t));
            } else {
                A64_LOGE("mprotect failed with errno = %d, p = %p, size = %zu",
                         errno, original, 5 * sizeof(uint32_t));
                trampoline = NULL;
            }
        } else {
            /*
             * 近距离跳转: 只需一条 B 指令
             *
             * 这是最理想的情况, 只需覆盖一条指令(4字节), 对原函数影响最小。
             * 当替换函数与原函数在同一个共享库中或相邻库中时通常会走这个分支。
             */
            if (trampoline) {
                if (rwx_size < 1u * 10u) {
                    A64_LOGE("rwx size is too small to hold %u bytes backup instructions!", 1u * 10u);
                    return NULL;
                }
                __fix_instructions(original, 1, trampoline);
            }

            if (__make_rwx(original, 1 * sizeof(uint32_t)) == 0) {
                /*
                 * 使用原子比较交换来写入跳转指令
                 *
                 * __sync_cmpswap 确保写入的原子性: 如果 original[0] 的当前值
                 * 等于 *original(即我们之前读取的值), 则将其替换为新的 B 指令。
                 *
                 * 这可以避免与其他线程的竞争条件, 虽然在 Hook 场景中竞争不太常见,
                 * 但这是一个好习惯。
                 */
                __sync_cmpswap(original, *original, 0x14000000u | (pc_offset & mask));
                __flush_cache(symbol, 1 * sizeof(uint32_t));

                A64_LOGI("inline hook %p->%p successfully! %zu bytes overwritten",
                         symbol, replace, 1 * sizeof(uint32_t));
            } else {
                A64_LOGE("mprotect failed with errno = %d, p = %p, size = %zu",
                         errno, original, 1 * sizeof(uint32_t));
                trampoline = NULL;
            }
        }

        return trampoline;
    }

    //-------------------------------------------------------------------------

    /*
     * A64HookFunction: 使用内置跳板池的 Hook 接口
     *
     * 这是面向用户的主要接口。它自动从跳板池分配槽位, 简化了使用流程。
     *
     * @param symbol:  要 Hook 的目标函数
     * @param replace: 替换函数
     * @param result:  输出参数, 返回跳板地址用于调用原函数。传 NULL 表示不需要调用原函数。
     */
    A64_JNIEXPORT void A64HookFunction(void *const symbol, void *const replace, void **result)
    {
        void *trampoline = NULL;

        if (result != NULL) {
            // 用户需要调用原函数, 分配跳板
            trampoline = FastAllocateTrampoline();
            *result = trampoline;
            if (trampoline == NULL) return;  // 分配失败
        }

        /*
         * Android 10 及以上版本的兼容性处理
         *
         * 从 Android 10 开始, 系统默认将 .text 段映射为只读+可执行(R-X),
         * 不再是之前的可读+可写+可执行(RWX)。这是一个安全加固措施。
         *
         * 因此在修改原函数入口之前, 需要先调用 mprotect 添加写权限。
         * 这里预先调用一次 __make_rwx 确保后续的写入操作能成功。
         *
         * 注意: 5 * sizeof(size_t) = 40 字节, 比实际需要的稍大一些, 以确保覆盖。
         */
        __make_rwx(symbol, 5 * sizeof(size_t));

        trampoline = A64HookFunctionV(symbol, replace, trampoline, A64_MAX_INSTRUCTIONS * 10u);

        if (trampoline == NULL && result != NULL) {
            *result = NULL;  // Hook 失败, 清空结果
        }
    }
}

#endif // defined(__aarch64__)
