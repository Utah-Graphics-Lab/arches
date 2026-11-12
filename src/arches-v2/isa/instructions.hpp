#pragma once

#include "stdafx.hpp"

#include "shared-utils.hpp"
#include "units/trax/unit-tp.hpp"
#include "units/trax/unit-rt-core.hpp"
#include "trax-kernel/include.hpp"
#include "trax-kernel/intersect.hpp"
#include "units/unit-texture.hpp"

namespace Arches
{

	namespace ISA
	{
		namespace RISCV
		{
			namespace TRaX
			{

				// see the opcode map for details
				const inline InstructionInfo isa_custom0_000_imm[8] =
					{
						InstructionInfo(0x0, "fchthrd", InstrType::CUSTOM0, Encoding::U, RegFile::INT, MEM_REQ_DECL {
		MemoryRequest req;
		req.type = MemoryRequest::Type::LOAD;
		req.size = sizeof(uint32_t);
		req.dst.push(DstReg(instr.rd, RegType::UINT32).u9, 9);
		req.vaddr = 0x0ull;
		return req; }),

							//box-intersection instruction
						InstructionInfo(0x1, "boxisect", InstrType::CUSTOM1, Encoding::U, RegFile::FLOAT, EXEC_DECL {
		Register32 * fr = unit->float_regs->registers;

		rtm::Ray ray;
		rtm::vec3 inv_d;
		ray.o.x = fr[0].f32;
		ray.o.y = fr[1].f32;
		ray.o.z = fr[2].f32;
		ray.t_min = fr[3].f32;
		inv_d.x = fr[4].f32;
		inv_d.y = fr[5].f32;
		inv_d.z = fr[6].f32;
		ray.t_max = fr[7].f32;

		rtm::AABB aabb;
		aabb.min.x = fr[8].f32;
		aabb.min.y = fr[9].f32;
		aabb.min.z = fr[10].f32;
		aabb.max.x = fr[11].f32;
		aabb.max.y = fr[12].f32;
		aabb.max.z = fr[13].f32;

		unit->float_regs->registers[instr.u.rd].f32 = rtm::intersect(aabb, ray, inv_d); }),

							//triangle-intersection instruction
						InstructionInfo(0x2, "triisect", InstrType::CUSTOM2, Encoding::U, RegFile::FLOAT, EXEC_DECL {
		Register32 * fr = unit->float_regs->registers;

		rtm::Ray ray;
		ray.o.x = fr[0].f32;
		ray.o.y = fr[1].f32;
		ray.o.z = fr[2].f32;
		ray.t_min = fr[3].f32;
		ray.d.x = fr[4].f32;
		ray.d.y = fr[5].f32;
		ray.d.z = fr[6].f32;
		ray.t_max = fr[7].f32;

		rtm::Triangle tri;
		tri.vrts[0].x = fr[8].f32;
		tri.vrts[0].y = fr[9].f32;
		tri.vrts[0].z = fr[10].f32;
		tri.vrts[1].x = fr[11].f32;
		tri.vrts[1].y = fr[12].f32;
		tri.vrts[1].z = fr[13].f32;
		tri.vrts[2].x = fr[14].f32;
		tri.vrts[2].y = fr[15].f32;
		tri.vrts[2].z = fr[16].f32;

		rtm::Hit hit;
		hit.t = fr[17].f32;
		hit.bc[0] = fr[18].f32;
		hit.bc[1] = fr[19].f32;
		hit.id = fr[20].u32;

		rtm::intersect(tri, ray, hit);

		fr[17].f32 = hit.t;
		fr[18].f32 = hit.bc[0];
		fr[19].f32 = hit.bc[1];
		fr[20].u32 = hit.id; }),
				};

					//implementation of "traceray" instruction
				const inline InstructionInfo isa_custom0_funct3[8] =
					{
						InstructionInfo(0x0, META_DECL { return isa_custom0_000_imm[instr.u.imm_31_12 >> 3]; }),
						InstructionInfo(0x1, IMPL_NONE),
						InstructionInfo(0x2, IMPL_NONE),
						InstructionInfo(0x3, IMPL_NONE),
						InstructionInfo(0x4, IMPL_NONE),
						InstructionInfo(0x5, "traceray", InstrType::CUSTOM7, Encoding::I, RegFile::FLOAT, MEM_REQ_DECL {
		MemoryRequest mem_req;
		mem_req.type = MemoryRequest::Type::STORE;
		mem_req.size = sizeof(rtm::Ray);
		mem_req.dst.push(DstReg(instr.rd, RegType::FLOAT32).u9, 9);
		mem_req.vaddr = 0xdeadbeefull;

		Register32* fr = unit->float_regs->registers;
		for(uint i = 0; i < sizeof(rtm::Ray) / sizeof(float); ++i)
			((float*)mem_req.data)[i] = fr[instr.i.rs1 + i].f32;

		return mem_req; }),

							//2D sampling instruction
						InstructionInfo(0x6, "sample2d", InstrType::CUSTOM3, Encoding::I, RegFile::FLOAT, MEM_REQ_DECL {
		Register32* fr = unit->float_regs->registers;

		MemoryRequest mem_req;
		mem_req.type = MemoryRequest::Type::STORE;
		mem_req.size = 8;
		mem_req.dst.push(DstReg(instr.rd, RegType::FLOAT32).u9, 9);
		mem_req.vaddr = fr[instr.i.rs1].u32;

		((float*)mem_req.data)[0] = fr[instr.i.rs1 + 1].f32;
		((float*)mem_req.data)[1] = fr[instr.i.rs1 + 2].f32;

		//fr[instr.i.rd + 0].f32 = fr[instr.i.rs1 + 1].f32;
		//fr[instr.i.rd + 1].f32 = fr[instr.i.rs1 + 2].f32;
		//fr[instr.i.rd + 2].f32 = 0.5f;
		//fr[instr.i.rd + 3].f32 = 1.0f;

		return mem_req; }),
				};

				const inline InstructionInfo custom0 = {CUSTOM_OPCODE0, META_DECL{return isa_custom0_funct3[instr.i.funct3];
			}
		};

	}
}
}
}