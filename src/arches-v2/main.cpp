#include "stdafx.hpp"

#include "shared-utils.hpp"
#include "units/trax/unit-tp.hpp"
#include "units/trax/unit-rt-core.hpp"
#include "trax-kernel/include.hpp"
#include "trax-kernel/intersect.hpp"
#include "units/unit-texture.hpp"
#include "util/live-stream.hpp"

namespace Arches {

namespace ISA {
namespace RISCV {
namespace TRaX {

//see the opcode map for details
const static InstructionInfo isa_custom0_000_imm[8] =
{
	InstructionInfo(0x0, "fchthrd", InstrType::CUSTOM0, Encoding::U, RegFile::INT, MEM_REQ_DECL
	{
		MemoryRequest req;
		req.type = MemoryRequest::Type::LOAD;
		req.size = sizeof(uint32_t);
		req.dst.push(DstReg(instr.rd, RegType::UINT32).u9, 9);
		req.vaddr = 0x0ull;
		return req;
	}),
	InstructionInfo(0x1, "boxisect", InstrType::CUSTOM1, Encoding::U, RegFile::FLOAT, EXEC_DECL
	{
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

		unit->float_regs->registers[instr.u.rd].f32 = rtm::intersect(aabb, ray, inv_d);
	}),
	InstructionInfo(0x2, "triisect", InstrType::CUSTOM2, Encoding::U, RegFile::FLOAT, EXEC_DECL
	{
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
		fr[20].u32 = hit.id;
	}),
	InstructionInfo(0x3, "BaryInterp3", InstrType::CUSTOM4, Encoding::U, RegFile::FLOAT, EXEC_DECL
	{
		Register32* fr = unit->float_regs->registers;

		rtm::vec3 a0(fr[0].f32, fr[1].f32, fr[2].f32);
		rtm::vec3 a1(fr[3].f32, fr[4].f32, fr[5].f32);
		rtm::vec3 a2(fr[6].f32, fr[7].f32, fr[8].f32);
		rtm::vec2 bc(fr[9].f32, fr[10].f32);

		rtm::vec3 a = a0 * bc.x + a1 * bc.y + a2 * (1.0f - bc.x - bc.y);

		fr[28].f32 = a.x;
		fr[29].f32 = a.y;
		fr[30].f32 = a.z;
	})
};

const static InstructionInfo isa_custom0_funct3[8] =
{
	InstructionInfo(0x0, META_DECL{return isa_custom0_000_imm[instr.u.imm_31_12 >> 3]; }),
	InstructionInfo(0x1, IMPL_NONE),
	InstructionInfo(0x2, IMPL_NONE),
	InstructionInfo(0x3, IMPL_NONE),
	InstructionInfo(0x4, IMPL_NONE),
	InstructionInfo(0x5, "traceray", InstrType::CUSTOM7, Encoding::I, RegFile::FLOAT, MEM_REQ_DECL
	{
		MemoryRequest mem_req;
		mem_req.type = MemoryRequest::Type::STORE;
		mem_req.size = sizeof(rtm::Ray);
		mem_req.dst.push(DstReg(instr.rd, RegType::FLOAT32).u9, 9);
		mem_req.vaddr = 0xdeadbeefull;

		Register32* fr = unit->float_regs->registers;
		for(uint i = 0; i < sizeof(rtm::Ray) / sizeof(float); ++i)
			((float*)mem_req.data)[i] = fr[instr.i.rs1 + i].f32;

		return mem_req;
	}),
	InstructionInfo(0x6, "sample2d", InstrType::CUSTOM3, Encoding::I, RegFile::FLOAT, MEM_REQ_DECL
	{
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

		return mem_req;
	}),
};

const static InstructionInfo custom0(CUSTOM_OPCODE0, META_DECL{return isa_custom0_funct3[instr.i.funct3]; });

}
}
}

namespace TRaX {

typedef Units::UnitDRAMRamulator UnitDRAM;
typedef Units::UnitCache UnitL3Cache;
typedef Units::UnitCache UnitL2Cache;
typedef Units::UnitCache UnitL1Cache;
typedef rtm::FTB PrimBlocks;
typedef Units::TRaX::UnitRTCore<rtm::CWBVH::Node, PrimBlocks> UnitRTCore;

static TRaXKernelArgs initilize_buffers(Units::UnitMainMemoryBase** drams, const Units::UnitCrossbar& xbar, paddr_t& heap_address, const SimulationConfig& sim_config, uint page_size)
{
	std::string scene_name = sim_config.get_string("scene-name");
	std::string project_folder = get_project_folder_path();
	std::string datasets_folder = sim_config.get_string("dataset-dir") + "/";
	std::string cache_folder = sim_config.get_string("dataset-dir") + "/cache/";

	TRaXKernelArgs args{};
	args.framebuffer_width = sim_config.get_int("framebuffer-width");
	args.framebuffer_height = sim_config.get_int("framebuffer-height");
	args.framebuffer_size = args.framebuffer_width * args.framebuffer_height;
	heap_address = align_to(page_size, heap_address);
	args.framebuffer = reinterpret_cast<uint32_t*>(heap_address);
	heap_address += args.framebuffer_size * sizeof(uint32_t);

	args.pregen_rays = sim_config.get_int("pregen-rays");
	uint pregen_bounce = sim_config.get_int("pregen-bounce");

	args.light_dir = rtm::normalize(rtm::vec3(4.5f, 42.5f, 5.0f));
	args.camera = sim_config.camera;

	rtm::Mesh mesh(datasets_folder + scene_name + ".obj");
	rtm::CWBVH bvh(mesh, (cache_folder + scene_name + ".bvh").c_str(), sim_config.get_int("bvh-preset"), sim_config.get_int("bvh-merging"));

	std::vector<rtm::Ray> rays(args.framebuffer_size);
	if(args.pregen_rays)
	{
		std::string ray_file = scene_name + "-" + std::to_string(args.framebuffer_width) + "-" + std::to_string(pregen_bounce) + ".rays";
	#if USE_HECWBVH_V1
		pregen_rays(&bvh.nodes[0], &bvh.nodes[0].ftb, mesh, args.framebuffer_width, args.framebuffer_height, args.camera, pregen_bounce, rays);
	#else
		pregen_rays(&bvh.nodes[0], &bvh.ftbs[0], mesh, args.framebuffer_width, args.framebuffer_height, args.camera, pregen_bounce, rays);
	#endif
		args.rays = write_vector(drams, xbar, 256, rays, heap_address);
	}

	args.materials = write_vector(drams, xbar, 256, mesh.materials, heap_address);

	args.nodes = write_vector(drams, xbar, 256, bvh.nodes, heap_address);

#if USE_HECWBVH_V1
	args.ftbs = (rtm::FTB*)args.nodes;
#else 
	args.ft_blocks = write_vector(drams, xbar, 256, bvh.ftbs, heap_address);
#endif

	args.vertex_indices = write_vector(drams, xbar, 256, mesh.vertex_indices, heap_address);
	args.normal_indices = write_vector(drams, xbar, 256, mesh.normal_indices, heap_address);
	args.tex_coord_indices = write_vector(drams, xbar, 256, mesh.tex_coord_indices, heap_address);

	args.vertices = write_vector(drams, xbar, 256, mesh.vertices, heap_address);
	args.normals = write_vector(drams, xbar, 256, mesh.normals, heap_address);
	args.tex_coords = write_vector(drams, xbar, 256, mesh.tex_coords, heap_address);

	args.material_indices = write_vector(drams, xbar, 256, mesh.material_indices, heap_address);

	for(uint32_t i = 0; i < mesh.materials.size(); ++i)
	{
		if(!mesh.materials[i].use_am)
			continue;
		Texture2D& tex = mesh.materials[i].albedo_texture;
		Texture2D::Texel* dev_tex = write_array(drams, xbar, 256, tex.texels, tex.width * tex.height, heap_address);
		free(tex.texels);
		tex.texels = dev_tex;
	}

	paddr_t mat_addr = (paddr_t)args.materials;
	args.materials = write_vector(drams, xbar, 256, mesh.materials, mat_addr);

	for(uint32_t i = 0; i < mesh.materials.size(); ++i)
		mesh.materials[i].albedo_texture.texels = nullptr;  // to not free device memory textures

	size_t temp = TRAX_KERNEL_ARGS_ADDRESS;
	write_array(drams, xbar, 256, (uint8_t*)&args, sizeof(TRaXKernelArgs), temp);
	return args;
}

static void run_sim_trax(SimulationConfig& sim_config)
{
	std::string project_folder_path = get_project_folder_path();

#if 0 //RTX 4090 ish
	//Compute
	double core_clock = 2235.0e6;
	double dram_clock = 5250.0e6;
	uint64_t stack_size = 512;
	uint num_tms = 128;
	uint num_tps = 128;
	uint num_threads = 12;

	//Memory
	uint64_t block_size = CACHE_BLOCK_SIZE;
	uint num_partitions = 12;
	uint partition_stride = 1 << 12;

	//DRAM
	UnitDRAM::Configuration dram_config;
	dram_config.config_path = get_config_folder_path() + "gddr6x_21000_config.yaml";
	dram_config.size = 1ull << 30; //1GB per partition
	dram_config.clock_ratio = dram_clock / core_clock;
	dram_config.latency = 254;

	//L3$
	UnitL3Cache::Configuration l3_config;
	l3_config.level = 3;
	l3_config.block_prefetch = true;
	l3_config.miss_alloc = true;
	l3_config.size = 6 << 20;
	l3_config.associativity = 16;
	l3_config.policy = Units::UnitCacheBase::Policy::LRU;
	l3_config.num_slices = 6;
	l3_config.crossbar_width = l3_config.num_slices;
	l3_config.num_mshr = 256;
	l3_config.num_subentries = 4;
	l3_config.latency = 480;

	UnitL3Cache::PowerConfig l3_power_config;

    //L2$
    UnitL2Cache::Configuration l2_config;
    l2_config.level = 2;
    l2_config.block_prefetch = true;
    l2_config.miss_alloc = true;
    l2_config.size = 6 << 20;
    l2_config.associativity = 16;
    l2_config.policy = Units::UnitCacheBase::Policy::LRU;
    l2_config.num_banks = 6;
    l2_config.crossbar_width = l2_config.num_banks;
    l2_config.num_mshr = 256;
    l2_config.num_subentries = 4;
    l2_config.latency = 187;

	UnitL2Cache::PowerConfig l2_power_config;

    Units::UnitCrossbar::Configuration xbar_config;
    xbar_config.num_slices = l3_config.num_slices;
    xbar_config.slice_stride = l3_config.block_size;
    xbar_config.num_partitions = num_partitions;
    xbar_config.partition_stride = partition_stride;

	//L1d$
	UnitL1Cache::Configuration l1d_config;
	l1d_config.level = 1;
	l1d_config.miss_alloc = true;
	l1d_config.size = 128 << 10;
	l1d_config.associativity = 32;
	l1d_config.policy = Units::UnitCacheBase::Policy::LRU;
	l1d_config.num_banks = 8;
	l1d_config.crossbar_width = l1d_config.num_banks;
	l1d_config.num_mshr = 512;
	l1d_config.num_subentries = 16;
	l1d_config.latency = 39;

	UnitL1Cache::PowerConfig l1d_power_config;

	UnitRTCore::Configuration rtc_config;
	rtc_config.max_rays = 64;
	rtc_config.num_cache_ports = 4;

#elif 0 //RTX 3070 ish
	//Compute
	double core_clock = 1500.0e6;
	uint64_t stack_size = 512;
	uint num_tms = 46;
	uint num_tps = 128;
	uint num_threads = 12;

	//Memory
	double dram_clock = 3500.0e6;
	uint num_partitions = 8;
	uint partition_stride = 1 << 12;

	//DRAM
	UnitDRAM::Configuration dram_config;
	dram_config.config_path = get_config_folder_path() + "gddr6_14000_config.yaml";
	dram_config.size = 1ull << 30; //1GB per partition
	dram_config.clock_ratio = dram_clock / core_clock;
	dram_config.latency = 254;

	//L3$
	UnitL3Cache::Configuration l3_config;
	l3_config.level = 3;
	l3_config.miss_alloc = true;
	l3_config.size = 512 << 10;
	l3_config.associativity = 16;
	l3_config.policy = Units::UnitCacheBase::Policy::LRU;
	l3_config.num_slices = 4;
	l3_config.crossbar_width = l3_config.num_slices;
	l3_config.num_mshr = 192;
	l3_config.num_subentries = 4;
	l3_config.latency = 480;

	UnitL3Cache::PowerConfig l3_power_config;

    //L2$
    UnitL2Cache::Configuration l2_config;
    l2_config.level = 2;
    l2_config.miss_alloc = true;
    l2_config.size = 512 << 10;
    l2_config.associativity = 16;
    l2_config.policy = Units::UnitCacheBase::Policy::LRU;
	l2_config.num_banks = 4;
    l2_config.crossbar_width = l2_config.num_banks;
    l2_config.num_mshr = 192;
    l2_config.num_subentries = 4;
    l2_config.latency = 187;

	UnitL2Cache::PowerConfig l2_power_config;

    Units::UnitCrossbar::Configuration xbar_config;
    xbar_config.num_slices = l3_config.num_slices;
    xbar_config.slice_stride = l3_config.block_size;
    xbar_config.num_partitions = num_partitions;
    xbar_config.partition_stride = partition_stride;

	//L1d$
	UnitL1Cache::Configuration l1d_config;
	l1d_config.level = 1;
	l1d_config.miss_alloc = true;
	l1d_config.size = 128 << 10;
	l1d_config.associativity = 32;
	l1d_config.policy = Units::UnitCacheBase::Policy::FIFO;
	l1d_config.num_banks = 4;
	l1d_config.crossbar_width = l1d_config.num_banks;
	l1d_config.num_mshr = 384;
	l1d_config.num_subentries = 48;
	l1d_config.latency = 39;

	UnitL1Cache::PowerConfig l1d_power_config;

	UnitRTCore::Configuration rtc_config;
	rtc_config.max_rays = 128;
	rtc_config.num_cache_ports = 4;

#elif 1 //Turing spec
#if 1 //RTX 2080
    double core_clock = 1515.0e6;
    uint num_threads = 8;
    uint num_tps = 64;
    uint num_tms = 64;// 46;
    uint64_t stack_size = 1024;

	double dram_clock = 3500.0e6;
	uint num_partitions = 8;
	uint partition_stride = 1 << 12;
#else //RTX 2060
	double core_clock = 1365.0e6;
	uint num_threads = 1;
	uint num_tps = 64;
	uint num_tms = 30;
	uint64_t stack_size = 512;

	double dram_clock = 3500.0e6;
	uint num_partitions = 6;
	uint partition_stride = 1 << 12;
#endif

	//DRAM
	UnitDRAM::Configuration dram_config;
	dram_config.config_path = get_config_folder_path() + "gddr6_14000_config.yaml";
	dram_config.size = 1ull << 30; //1GB
	dram_config.clock_ratio = dram_clock / core_clock;
	dram_config.latency = 92;

	//L3$
	UnitL3Cache::Configuration l3_config;
	l3_config.level = 3;
	l3_config.miss_alloc = true;
	l3_config.size = 4096 << 10;
	l3_config.associativity = 16;
	l3_config.num_slices = 4;
	l3_config.crossbar_width = l3_config.num_slices;
	l3_config.num_mshr = 256;
	l3_config.num_subentries = 16;
	l3_config.latency = 480;

	UnitL3Cache::PowerConfig l3_power_config;

    //L2$
    UnitL2Cache::Configuration l2_config;
    l2_config.level = 2;
    l2_config.miss_alloc = true;
    l2_config.size = 512 << 10;
    l2_config.associativity = 16;
    l2_config.num_banks = 4;
    //l2_config.num_slices = 1;
    l2_config.crossbar_width = l2_config.num_banks;
    l2_config.num_mshr = 256;
    l2_config.num_subentries = 16;
    l2_config.latency = 160;

	UnitL2Cache::PowerConfig l2_power_config;

    Units::UnitCrossbar::Configuration xbar_config;
    xbar_config.num_slices = l3_config.num_slices;
    xbar_config.slice_stride = l3_config.block_size;
    xbar_config.num_partitions = num_partitions;
    xbar_config.partition_stride = partition_stride;

	//L1d$
	UnitL1Cache::Configuration l1d_config;
	l1d_config.level = 1;
	l1d_config.miss_alloc = true;
	l1d_config.size = 64 << 10;
	l1d_config.associativity = 32;
	l1d_config.num_banks = 16;
	l1d_config.crossbar_width = l1d_config.num_banks;
	l1d_config.num_mshr = 256;
	l1d_config.num_subentries = 16;
	l1d_config.latency = 20;

	UnitL1Cache::PowerConfig l1d_power_config;

	Units::UnitTexture::Configuration tu_config;

	UnitRTCore::Configuration rtc_config;
	rtc_config.max_rays = 64;
	rtc_config.num_cache_ports = 4;
#else //TRaX 1.0
	double core_clock = 1000.0e6;
	uint num_threads = 1;
	uint num_tps = 32;
	uint num_tms = 32;
	uint num_rays = 32;
	uint64_t stack_size = 4096;

	double dram_clock = 2000.0e6;
	uint num_partitions = 4;
	uint partition_stride = 1 << 12;

	//DRAM
	UnitDRAM::Configuration dram_config;
	dram_config.config_path = get_config_folder_path() + "gddr6_pch_config.yaml";
	dram_config.size = 1ull << 30; //1GB
	dram_config.clock_ratio = dram_clock / core_clock;
	dram_config.latency = 1;
	//dram_config.latency = 56;

	//L3$
	UnitL3Cache::Configuration l3_config;
	l3_config.level = 3;
	l3_config.miss_alloc = false;
	l3_config.size = 256 << 10;
	l3_config.associativity = 16;
	l3_config.num_slices = 1;
	l3_config.crossbar_width = l3_config.num_slices;
	l3_config.num_mshr = 1;
	l3_config.num_subentries = 1;
	l3_config.latency = 10;

	UnitL3Cache::PowerConfig l3_power_config;

    //L2$
    UnitL2Cache::Configuration l2_config;
    l2_config.level = 2;
    l2_config.miss_alloc = false;
    l2_config.size = 256 << 10;
    l2_config.associativity = 16;
    l2_config.num_banks = 1;
    l2_config.crossbar_width = l2_config.num_banks;
    l2_config.num_mshr = 1;
    l2_config.num_subentries = 1;
    l2_config.latency = 10;

	UnitL2Cache::PowerConfig l2_power_config;

    Units::UnitCrossbar::Configuration xbar_config;
    xbar_config.num_slices = l3_config.num_slices;
    xbar_config.slice_stride = l3_config.block_size;
    xbar_config.num_partitions = num_partitions;
    xbar_config.partition_stride = partition_stride;

	//L1d$
	UnitL1Cache::Configuration l1d_config;
	l1d_config.level = 1;
	l1d_config.miss_alloc = false;
	l1d_config.size = 32 << 10;
	l1d_config.associativity = 4;
	l1d_config.num_banks = 8;
	l1d_config.crossbar_width = l1d_config.num_banks;
	l1d_config.num_mshr = 256;
	l1d_config.num_subentries = 16;
	l1d_config.latency = 1;

	UnitL1Cache::PowerConfig l1d_power_config;
#endif

	ELF elf(project_folder_path + "src/trax-kernel/riscv/kernel");

	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM0] = "FCHTHRD";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM1] = "BOXISECT";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM2] = "TRIISECT";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM3] = "SAMPLE2D";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM4] = "BARYINT3";
	ISA::RISCV::InstructionTypeNameDatabase::get_instance()[ISA::RISCV::InstrType::CUSTOM7] = "TRACERAY";
	ISA::RISCV::isa[ISA::RISCV::CUSTOM_OPCODE0] = ISA::RISCV::TRaX::custom0;

	uint num_sfus = static_cast<uint>(ISA::RISCV::InstrType::NUM_TYPES) * num_tms;

	Simulator simulator;
	std::vector<Units::UnitTP*> tps;
	std::vector<Units::UnitSFU*> sfus;
	std::vector<Units::UnitThreadScheduler*> thread_schedulers;
	std::vector<UnitRTCore*> rtcs;
	std::vector<Units::UnitTexture*> tus;
	std::vector<UnitL1Cache*> l1ds;
	std::vector<std::vector<Units::UnitBase*>> unit_tables; unit_tables.reserve(num_tms);
	std::vector<std::vector<Units::UnitSFU*>> sfu_lists; sfu_lists.reserve(num_tms);
	std::vector<std::vector<Units::UnitMemoryBase*>> mem_lists; mem_lists.reserve(num_tms);

    //construct memory partitions
    std::vector<UnitDRAM*> drams;
    std::vector<UnitL3Cache*> l3s;
    dram_config.num_ports = l3_config.num_slices;
    l3_config.num_ports = l3_config.num_slices;
    for (uint i = 0; i < num_partitions; ++i)
    {
        drams.push_back(_new UnitDRAM(dram_config));
        simulator.register_unit(drams.back());

        l3_config.mem_higher_port = 0;
        l3_config.mem_highers = { drams.back() };
        l3s.push_back(_new UnitL3Cache(l3_config));
        simulator.register_unit(l3s.back());
        simulator.new_unit_group();

        xbar_config.mem_highers.push_back(l3s.back());
    }

    xbar_config.num_clients = num_tms / 8;
    Units::UnitCrossbar xbar(xbar_config);
    simulator.register_unit(&xbar);
    simulator.new_unit_group();

	std::vector<UnitL2Cache*> l2s;
	l2_config.mem_highers = { &xbar }; // L1 (lower) <-> L2 <-> xbar (higher)
    l2_config.num_ports = 8; // can accept requests from 8 L1s
    l2_config.crossbar_width = 8;
	// construct 1 L2 module for every 8 L1s
	for (int l2_index = 0; l2_index < num_tms / 8; ++l2_index) {
		l2_config.mem_higher_port = l2_index; // xbar port id to connect to
        l2s.push_back(_new UnitL2Cache(l2_config));
        simulator.register_unit(l2s.back());
    }
    simulator.new_unit_group();

	std::vector<uint8_t> vec_mem;
	vec_mem.resize(256 << 20);

	paddr_t heap_address = elf.load(vec_mem.data());
	for(uint addr = 0; addr < heap_address; addr += partition_stride)
		drams[xbar.get_partition(addr)]->direct_write(vec_mem.data() + addr, partition_stride, xbar.strip_partition_bits(addr));

	TRaXKernelArgs kernel_args = initilize_buffers((Units::UnitMainMemoryBase**)drams.data(), xbar, heap_address, sim_config, partition_stride);
	heap_address = align_to(partition_stride, heap_address);

	for(uint addr = 0; addr < (256 << 20); addr += partition_stride)
		drams[xbar.get_partition(addr)]->direct_read(vec_mem.data() + addr, partition_stride, xbar.strip_partition_bits(addr));

	//bool warm_l2 = false;
	//if(warm_l2)
	//{
	//	paddr_t start = (paddr_t)kernel_args.nodes & ~(1 - partition_stride);
	//	paddr_t end = start + l2_config.size * num_partitions;
	//	for(paddr_t block_addr = end - l2_config.block_size; block_addr >= start; block_addr -= l2_config.block_size)
	//		l2s[xbar.get_partition(block_addr)]->direct_write(xbar.strip_partition_bits(block_addr), device_mem + block_addr);
	//}

	//bool deserialize_l2 = false, serialize_l2 = !deserialize_l2;
	//if(deserialize_l2)
	//	for(uint i = 0; i < num_partitions; ++i)
	//		serialize_l2 = !l2s[i]->deserialize("l2-p" + std::to_string(i) + ".bin", *drams[i]);

	Units::UnitAtomicRegfile atomic_regs(num_tms);
	simulator.register_unit(&atomic_regs);
	simulator.new_unit_group();

    l1d_config.num_ports = num_tps;
    l1d_config.mem_highers = { &xbar };
#if TRAX_USE_RT_CORE
	l1d_config.num_ports += num_tps;
	l1d_config.crossbar_width *= 2;
#endif

	for(uint tm_index = 0; tm_index < num_tms; ++tm_index)
	{
		std::vector<Units::UnitBase*> unit_table((uint)ISA::RISCV::InstrType::NUM_TYPES, nullptr);
		std::vector<Units::UnitMemoryBase*> mem_list;
		std::vector<Units::UnitSFU*> sfu_list;

        l1d_config.mem_highers = { l2s[tm_index / 8] }; // L1 <-> L2 (higher)
        l1d_config.mem_higher_port = tm_index % 8;
        l1ds.push_back(new UnitL1Cache(l1d_config));
        simulator.register_unit(l1ds.back());
        mem_list.push_back(l1ds.back());
        unit_table[(uint)ISA::RISCV::InstrType::LOAD] = l1ds.back();
        unit_table[(uint)ISA::RISCV::InstrType::STORE] = l1ds.back();

		thread_schedulers.push_back(_new  Units::UnitThreadScheduler(num_tps, tm_index, &atomic_regs, 32));
		simulator.register_unit(thread_schedulers.back());
		mem_list.push_back(thread_schedulers.back());
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM0] = thread_schedulers.back();

		//sfu_list.push_back(_new Units::UnitSFU(num_tps_per_tm, 2, 1, num_tps_per_tm));
		//simulator.register_unit(sfu_list.back());
		//unit_table[(uint)ISA::RISCV::InstrType::FADD] = sfu_list.back();
		//unit_table[(uint)ISA::RISCV::InstrType::FMUL] = sfu_list.back();
		//unit_table[(uint)ISA::RISCV::InstrType::FFMAD] = sfu_list.back();

		sfu_list.push_back(_new Units::UnitSFU(num_tps / 8, 1, 1, num_tps));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::IMUL] = sfu_list.back();
		unit_table[(uint)ISA::RISCV::InstrType::IDIV] = sfu_list.back();

		sfu_list.push_back(_new Units::UnitSFU(num_tps / 16, 6, 1, num_tps));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::FDIV] = sfu_list.back();
		unit_table[(uint)ISA::RISCV::InstrType::FSQRT] = sfu_list.back();

	#if TRAX_USE_HARDWARE_INTERSECTORS
		sfu_list.push_back(_new Units::UnitSFU(2, 3, 1, num_tps_per_tm));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM1] = sfu_list.back();

		sfu_list.push_back(_new Units::UnitSFU(1, 22, 8, num_tps_per_tm));
		simulator.register_unit(sfu_list.back());
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM2] = sfu_list.back();
	#endif

		for(auto& sfu : sfu_list)
			sfus.push_back(sfu);

		//l1s_config.mem_higher_port = tm_index * 2 + 1;
		//l1ss.push_back(new Units::UnitStreamCache(l1s_config));
		//simulator.register_unit(l1ss.back());

		tu_config.num_clients = num_tps;
		tu_config.cache = l1ds.back();
		tu_config.cache_port = num_tps + 1;
		tu_config.cheat_mem = vec_mem.data();

		tus.push_back(_new  Units::UnitTexture(tu_config));
		simulator.register_unit(tus.back());
		mem_list.push_back(tus.back());
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM3] = tus.back();

	#if TRAX_USE_RT_CORE
		rtc_config.num_clients = num_tps;
		rtc_config.node_base_addr = (paddr_t)kernel_args.nodes;
		rtc_config.tri_base_addr = (paddr_t)kernel_args.ft_blocks;
		rtc_config.cache = l1ds.back();
		rtc_config.cache_port = num_tps;
		rtc_config.cache_port_stride = num_tps / l1d_config.num_banks;

		rtcs.push_back(_new  UnitRTCore(rtc_config));
		simulator.register_unit(rtcs.back());
		mem_list.push_back(rtcs.back());
		unit_table[(uint)ISA::RISCV::InstrType::CUSTOM7] = rtcs.back();
	#endif

		unit_tables.emplace_back(unit_table);
		sfu_lists.emplace_back(sfu_list);
		mem_lists.emplace_back(mem_list);

		Units::UnitTP::Configuration tp_config;
		tp_config.tm_index = tm_index;
		tp_config.stack_size = stack_size;
		tp_config.cheat_memory = vec_mem.data();
		tp_config.unique_mems = &mem_lists.back();
		tp_config.unique_sfus = &sfu_lists.back();
		tp_config.num_threads = num_threads;
		for(uint tp_index = 0; tp_index < num_tps; ++tp_index)
		{
			tp_config.tp_index = tp_index;
			tp_config.unit_table = &unit_tables.back();
			tps.push_back(new Units::TRaX::UnitTP(tp_config));
			simulator.register_unit(tps.back());
		}

		simulator.new_unit_group();
	}

	printf("Starting TRaX\n");
	for(auto& tp : tps)
		tp->set_entry_point(elf.elf_header->e_entry.u64);

    //master logs
    UnitDRAM::Log dram_log;
    UnitL3Cache::Log l3_log;
    UnitL2Cache::Log l2_log;
    UnitL1Cache::Log l1d_log;
    Units::UnitTP::Log tp_log;

	UnitRTCore::Log rtc_log;
	Units::UnitTexture::Log tu_log;

    uint delta = sim_config.get_int("logging-interval");
    float delta_s = delta / core_clock;
    float delta_ns = delta_s * 1e9;
    float delta_dram_cycles = delta_s * dram_clock;
    float peak_dram_bandwidth = dram_clock / core_clock * num_partitions * 4 * 32 / 8;
    float peak_l3_bandwidth = num_partitions * l3_config.num_slices * MemoryRequest::MAX_SIZE;
    float peak_l2_bandwidth = l2s.size() * l2_config.num_banks * MemoryRequest::MAX_SIZE;
    float peak_l1d_bandwidth = num_tms * l1d_config.num_banks * MemoryRequest::MAX_SIZE;

	//live stream to an external visualizer
	LiveStream live_stream(sim_config.get_string("live-host"), (uint16_t)sim_config.get_int("live-port"));
	uint live_frame_interval = sim_config.get_int("live-frame-interval");
	bool print_intervals = sim_config.get_int("print-intervals") != 0;
	uint live_interval_index = 0;
	constexpr uint num_instr_types = (uint)ISA::RISCV::InstrType::NUM_TYPES;
	uint64_t live_tp_prev[3][num_instr_types] = {};
	std::vector<uint64_t> live_tp_issued_prev(tps.size(), 0);

	std::vector<uint8_t> live_framebuffer;
	auto read_framebuffer = [&]() -> const uint8_t*
	{
		paddr_t fb_start = (paddr_t)kernel_args.framebuffer;
		paddr_t fb_end = fb_start + (paddr_t)kernel_args.framebuffer_size * sizeof(uint32_t);
		paddr_t start = fb_start / partition_stride * partition_stride;
		live_framebuffer.resize(fb_end - start + partition_stride);
		for(paddr_t addr = start; addr < fb_end; addr += partition_stride)
			drams[xbar.get_partition(addr)]->direct_read(live_framebuffer.data() + (addr - start), partition_stride, xbar.strip_partition_bits(addr));
		return live_framebuffer.data() + (fb_start - start);
	};

	auto write_cache_log = [](JsonWriter& json, const char* key, const UnitL1Cache::Log& log)
	{
		json.begin_object(key)
			.value("hits", log.hits)
			.value("misses", log.misses)
			.value("half_misses", log.half_misses)
			.value("uncached_requests", log.uncached_requests)
			.value("mshr_stalls", log.mshr_stalls)
			.value("bytes_read", log.bytes_read)
			.value("tag_array_access", log.tag_array_access)
			.value("data_array_reads", log.data_array_reads)
			.value("data_array_writes", log.data_array_writes)
			.end_object();
	};

	auto write_cache_config = [](JsonWriter& json, const char* key, const UnitL1Cache::Configuration& config, uint num_units, float peak_bandwidth)
	{
		json.begin_object(key)
			.value("units", num_units)
			.value("size", config.size)
			.value("associativity", config.associativity)
			.value("slices", config.num_slices)
			.value("banks", config.num_banks)
			.value("mshrs", config.num_mshr)
			.value("latency", config.latency)
			.value("peak_bytes_per_cycle", (double)peak_bandwidth)
			.end_object();
	};

	if(live_stream.connected())
	{
		JsonWriter json;
		json.begin_object()
			.value("version", 1u)
			.value("arch", sim_config.get_string("arch-name"))
			.value("scene", sim_config.get_string("scene-name"))
			.value("core_clock_hz", core_clock)
			.value("dram_clock_hz", dram_clock)
			.value("logging_interval", delta)
			.value("num_tms", num_tms)
			.value("num_tps", num_tps)
			.value("num_threads", num_threads)
			.value("num_partitions", num_partitions)
			.value("framebuffer_width", kernel_args.framebuffer_width)
			.value("framebuffer_height", kernel_args.framebuffer_height)
			.value("has_rt_core", (uint)!rtcs.empty())
			.value("tms_per_l2", (uint)(num_tms / l2s.size()))
			.begin_object("dram")
				.value("units", (uint)drams.size())
				.value("peak_bytes_per_cycle", (double)peak_dram_bandwidth)
			.end_object();
		write_cache_config(json, "l3", l3_config, (uint)l3s.size(), peak_l3_bandwidth);
		write_cache_config(json, "l2", l2_config, (uint)l2s.size(), peak_l2_bandwidth);
		write_cache_config(json, "l1d", l1d_config, (uint)l1ds.size(), peak_l1d_bandwidth);
		json.begin_array("instr_types");
		for(uint i = 0; i < num_instr_types; ++i)
			json.value(nullptr, ISA::RISCV::InstructionTypeNameDatabase::get_instance()[(ISA::RISCV::InstrType)i]);
		json.end_array().end_object();
		live_stream.send(LiveStream::MessageType::HELLO, json.str());
	}

	//peak_dram_bandwidth = 32 * core_clock * num_partitions;
	//peak_l2_bandwidth = 32 * num_tms / 2;

	//per-unit counters for the visualizer's heatmaps; must run before delta_log resets the unit logs
	auto snapshot_units = [&]() -> std::string
	{
		auto write_caches = [](JsonWriter& json, const char* key, const std::vector<UnitL1Cache*>& caches)
		{
			std::vector<uint64_t> hits, half_misses, misses, mshr_stalls, bytes_read;
			for(auto& cache : caches)
			{
				hits.push_back(cache->log.hits);
				half_misses.push_back(cache->log.half_misses);
				misses.push_back(cache->log.misses);
				mshr_stalls.push_back(cache->log.mshr_stalls);
				bytes_read.push_back(cache->log.bytes_read);
			}
			json.begin_object(key)
				.counters("hits", hits)
				.counters("half_misses", half_misses)
				.counters("misses", misses)
				.counters("mshr_stalls", mshr_stalls)
				.counters("bytes_read", bytes_read)
				.end_object();
		};

		JsonWriter json;
		json.begin_object();
		write_caches(json, "l1d", l1ds);
		write_caches(json, "l2", l2s);
		write_caches(json, "l3", l3s);

		std::vector<uint64_t> dram_bytes_read, dram_loads, dram_stores, rays;
		for(auto& dram : drams)
		{
			dram_bytes_read.push_back(dram->log.bytes_read);
			dram_loads.push_back(dram->log.loads);
			dram_stores.push_back(dram->log.stores);
		}
		for(auto& rtc : rtcs)
			rays.push_back(rtc->log.rays);

		json.begin_object("dram")
				.counters("bytes_read", dram_bytes_read)
				.counters("loads", dram_loads)
				.counters("stores", dram_stores)
			.end_object()
			.begin_object("rtc")
				.counters("rays", rays)
			.end_object()
			.end_object();
		return json.str();
	};

	auto send_live_stats = [&](uint cycles, double wall_time, const std::string& units_json, const UnitDRAM::Log& dram_delta_log, const UnitL3Cache::Log& l3_delta_log, const UnitL2Cache::Log& l2_delta_log,
		const UnitL1Cache::Log& l1d_delta_log, const Units::UnitTexture::Log& tu_delta_log, const UnitRTCore::Log& rtc_delta_log)
	{
		//TP logs are not reset per interval (the final report and profile rely on them), so diff running totals instead
		uint64_t tp_totals[3][num_instr_types] = {};
		std::vector<uint64_t> tp_issued(tps.size());
		for(uint t = 0; t < tps.size(); ++t)
		{
			uint64_t issued = 0;
			for(uint i = 0; i < num_instr_types; ++i)
			{
				tp_totals[0][i] += tps[t]->log.instruction_counters[i];
				tp_totals[1][i] += tps[t]->log._resource_stall_counters[i];
				tp_totals[2][i] += tps[t]->log._data_stall_counters[i];
				issued += tps[t]->log.instruction_counters[i];
			}
			tp_issued[t] = issued - live_tp_issued_prev[t];
			live_tp_issued_prev[t] = issued;
		}
		uint64_t tp_delta[3][num_instr_types];
		for(uint j = 0; j < 3; ++j)
			for(uint i = 0; i < num_instr_types; ++i)
			{
				tp_delta[j][i] = tp_totals[j][i] - live_tp_prev[j][i];
				live_tp_prev[j][i] = tp_totals[j][i];
			}

		JsonWriter json;
		json.begin_object()
			.value("cycle", (int64_t)simulator.current_cycle)
			.value("cycles", cycles)
			.value("wall_time_s", wall_time)
			.value("threads_launched", (uint64_t)atomic_regs.iregs[0])
			.begin_object("dram")
				.value("loads", dram_delta_log.loads)
				.value("stores", dram_delta_log.stores)
				.value("bytes_read", dram_delta_log.bytes_read)
				.value("bytes_written", dram_delta_log.bytes_written)
				.value("unique_loads", dram_delta_log.unique_loads)
				.value("unique_rows", dram_delta_log.unique_rows)
			.end_object();
		write_cache_log(json, "l3", l3_delta_log);
		write_cache_log(json, "l2", l2_delta_log);
		write_cache_log(json, "l1d", l1d_delta_log);
		json.begin_object("tu")
				.value("texels", tu_delta_log.texels)
				.value("loads", tu_delta_log.loads)
			.end_object()
			.begin_object("rtc")
				.value("rays", rtc_delta_log.rays)
				.value("nodes", rtc_delta_log.nodes)
				.value("strips", rtc_delta_log.strips)
				.value("tris", rtc_delta_log.tris)
				.value("restarts", rtc_delta_log.restarts)
				.value("hits_returned", rtc_delta_log.hits_returned)
			.end_object()
			.begin_object("tp")
				.counters("issued", tp_delta[0], num_instr_types)
				.counters("resource_stalls", tp_delta[1], num_instr_types)
				.counters("data_stalls", tp_delta[2], num_instr_types)
				.counters("issued_per_tp", tp_issued) //TM-major: index = tm * num_tps + tp
			.end_object()
			.raw("units", units_json)
			.end_object();
		live_stream.send(LiveStream::MessageType::STATS, json.str());
	};

    auto start = std::chrono::high_resolution_clock::now();
    simulator.execute(delta, [&]() -> void
        {
			std::string units_json = live_stream.connected() ? snapshot_units() : std::string();
            UnitDRAM::Log dram_delta_log = delta_log(dram_log, drams);
            UnitL3Cache::Log l3_delta_log = delta_log(l3_log, l3s);
            UnitL2Cache::Log l2_delta_log = delta_log(l2_log, l2s);
            UnitL1Cache::Log l1d_delta_log = delta_log(l1d_log, l1ds);
            UnitRTCore::Log rtc_delta_log = delta_log(rtc_log, rtcs);
            Units::UnitTexture::Log tu_delta_log = delta_log(tu_log, tus);

		double simulation_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() / 1000.0;

		if(print_intervals)
		{
            printf("                            \n");
            printf("Cycle: %lld                 \n", simulator.current_cycle);
            printf("Threads Launched: %d        \n", atomic_regs.iregs[0]);
            printf("Simulation rate: %.2f KHz\n", simulator.current_cycle / simulation_time / 1000.0);
            printf("                            \n");
            printf("DRAM Read: %8.1f GB/s  (%.2f%%)\n", (float)dram_delta_log.bytes_read / delta_ns, 100.0f * dram_delta_log.bytes_read / delta / peak_dram_bandwidth);
            printf(" L3$ Read: %8.1f B/clk (%.2f%%)\n", (float)l3_delta_log.bytes_read / delta, 100.0f * l3_delta_log.bytes_read / delta / peak_l3_bandwidth);
            printf(" L2$ Read: %8.1f B/clk (%.2f%%)\n", (float)l2_delta_log.bytes_read / delta, 100.0f * l2_delta_log.bytes_read / delta / peak_l2_bandwidth);
            printf("L1d$ Read: %8.1f B/clk (%.2f%%)\n", (float)l1d_delta_log.bytes_read / delta, 100.0 * l1d_delta_log.bytes_read / delta / peak_l1d_bandwidth);
            printf("                            \n");
            printf(" L3$ Hit/Half/Miss: %3.1f%%/%3.1f%%/%3.1f%%\n", 100.0 * l3_delta_log.hits / l3_delta_log.get_total(), 100.0 * l3_delta_log.half_misses / l3_delta_log.get_total(), 100.0 * l3_delta_log.misses / l3_delta_log.get_total());
            printf(" L2$ Hit/Half/Miss: %3.1f%%/%3.1f%%/%3.1f%%\n", 100.0 * l2_delta_log.hits / l2_delta_log.get_total(), 100.0 * l2_delta_log.half_misses / l2_delta_log.get_total(), 100.0 * l2_delta_log.misses / l2_delta_log.get_total());
            printf("L1d$ Hit/Half/Miss: %3.1f%%/%3.1f%%/%3.1f%%\n", 100.0 * l1d_delta_log.hits / l1d_delta_log.get_total(), 100.0 * l1d_delta_log.half_misses / l1d_delta_log.get_total(), 100.0 * l1d_delta_log.misses / l1d_delta_log.get_total());
            printf("                            \n");
            printf(" L3$ Stalls: %0.2f%%\n", 100.0 * l3_delta_log.mshr_stalls / num_partitions / l3_config.num_slices / l3_config.num_banks / delta);
            printf(" L2$ Stalls: %0.2f%%\n", 100.0 * l2_delta_log.mshr_stalls / l2s.size() / l2_config.num_slices / l2_config.num_banks / delta);
            printf("L1d$ Stalls: %0.2f%%\n", 100.0 * l1d_delta_log.mshr_stalls / num_tms / l1d_config.num_banks / delta);
            printf("                            \n");
            printf("L3$  Occ: %0.2f%%\n", 100.0 * l3_delta_log.get_total() / num_partitions / l3_config.num_slices / l3_config.num_banks / delta);
            printf("L2$  Occ: %0.2f%%\n", 100.0 * l2_delta_log.get_total() / l2s.size() / l2_config.num_slices / l2_config.num_banks / delta);
            printf("L1d$ Occ: %0.2f%%\n", 100.0 * l1d_delta_log.get_total() / num_tms / l1d_config.num_banks / delta);
            printf("                            \n");

            tu_delta_log.print(delta, tus.size());

            if (!rtcs.empty())
            {
                printf("MRays/s: %.0f\n\n", rtc_delta_log.rays / delta_ns * 1000.0);
                rtc_delta_log.print(rtcs.size());
            }
		}

		if(live_stream.connected())
		{
			send_live_stats(delta, simulation_time, units_json, dram_delta_log, l3_delta_log, l2_delta_log, l1d_delta_log, tu_delta_log, rtc_delta_log);

			++live_interval_index;
			if(live_frame_interval != 0 && live_interval_index % live_frame_interval == 0)
				live_stream.send_frame(simulator.current_cycle, kernel_args.framebuffer_width, kernel_args.framebuffer_height, read_framebuffer());
		}
        });

	auto stop = std::chrono::high_resolution_clock::now();

	//send the final partial interval so the streamed intervals sum to the end-of-run totals
	uint tail_cycles = delta ? (uint)(simulator.current_cycle % delta) : 0;
	if(live_stream.connected() && tail_cycles != 0)
	{
		std::string units_json = snapshot_units();
		UnitDRAM::Log dram_delta_log = delta_log(dram_log, drams);
		UnitL3Cache::Log l3_delta_log = delta_log(l3_log, l3s);
		UnitL2Cache::Log l2_delta_log = delta_log(l2_log, l2s);
		UnitL1Cache::Log l1d_delta_log = delta_log(l1d_log, l1ds);
		UnitRTCore::Log rtc_delta_log = delta_log(rtc_log, rtcs);
		Units::UnitTexture::Log tu_delta_log = delta_log(tu_log, tus);
		double wall_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() / 1000.0;
		send_live_stats(tail_cycles, wall_time, units_json, dram_delta_log, l3_delta_log, l2_delta_log, l1d_delta_log, tu_delta_log, rtc_delta_log);
	}

	for(uint addr = 0; addr < (256 << 20); addr += partition_stride)
		drams[xbar.get_partition(addr)]->direct_read(vec_mem.data() + addr, partition_stride, xbar.strip_partition_bits(addr));

	//if(serialize_l2)
	//	for(uint i = 0; i < num_partitions; ++i)
	//		l2s[i]->serialize("l2-p" + std::to_string(i) + ".bin");

	cycles_t frame_cycles = simulator.current_cycle;
	double frame_time = frame_cycles / core_clock;
	double frame_time_ns = frame_time * 1e9;
	double simulation_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() / 1000.0;

	tp_log.print_profile(vec_mem.data());

	float total_power = 0.0f;
	for(auto& dram : drams)
	{
		dram->print_stats(4, frame_cycles);
		total_power += dram->total_power();
	}
	print_header("DRAM");
	delta_log(dram_log, drams);
	printf("DRAM Read: %.1f GB/s (%.2f%%)\n", (float)dram_log.bytes_read / frame_time_ns, 100.0f * dram_log.bytes_read / frame_cycles / peak_dram_bandwidth);
	dram_log.print(frame_cycles);

    print_header("L3$");
    delta_log(l3_log, l3s);
    printf(" L3$ Read: %.1f B/clk (%.2f%%)\n", (float)l3_log.bytes_read / frame_cycles, 100.0f * l3_log.bytes_read / frame_cycles / peak_l3_bandwidth);
    l3_log.print(frame_cycles);
    total_power += l3_log.print_power(l3_power_config, frame_time);
	
	print_header("L2$");
    delta_log(l2_log, l2s);
    printf(" L2$ Read: %.1f B/clk (%.2f%%)\n", (float)l2_log.bytes_read / frame_cycles, 100.0f * l2_log.bytes_read / frame_cycles / peak_l2_bandwidth);
    l2_log.print(frame_cycles);
    total_power += l2_log.print_power(l2_power_config, frame_time);

	print_header("L1d$");
	delta_log(l1d_log, l1ds);
	printf("L1d$ Read: %.1f B/clk (%.2f%%)\n", (float)l1d_log.bytes_read / frame_cycles, 100.0 * l1d_log.bytes_read / frame_cycles / peak_l1d_bandwidth);
	l1d_log.print(frame_cycles);
	total_power += l1d_log.print_power(l1d_power_config, frame_time);

	print_header("Texture Unit");
	delta_log(tu_log, tus);
	tu_log.print(frame_cycles, tus.size());

	print_header("TP");
	delta_log(tp_log, tps);
	tp_log.print(tps.size());

	if(!rtcs.empty())
	{
		print_header("RT Core");
		delta_log(rtc_log, rtcs);
		rtc_log.print(rtcs.size());
	}

	float total_energy = total_power * frame_time;

	print_header("Performance Summary");
	printf("Cycles: %lld\n", simulator.current_cycle);
	printf("Clock rate: %.0f MHz\n", core_clock / 1'000'000.0);
	printf("Frame time: %.3g ms\n", frame_time * 1000.0);
	if(!rtcs.empty()) printf("MRays/s: %.0f\n", rtc_log.rays / frame_time / 1'000'000.0);
	else              printf("MRays/s: %.0f\n", kernel_args.framebuffer_size / frame_time / 1'000'000.0);

	print_header("Power Summary");
	printf("Energy: %.2f mJ\n", total_power * frame_time * 1000.0);
	printf("Power: %.2f W\n", total_power);
	if(!rtcs.empty()) printf("MRays/J: %.2f\n", rtc_log.rays / total_energy / 1'000'000.0);
	else              printf("MRays/J: %.2f\n", kernel_args.framebuffer_size / total_energy / 1'000'000.0);

	print_header("Simulation Summary");
	printf("Simulation rate: %.2f KHz\n", simulator.current_cycle / simulation_time / 1000.0);
	printf("Simulation time: %.0f s\n", simulation_time);
	printf("MSIPS: %.2f\n", simulator.current_cycle * tps.size() / simulation_time / 1'000'000.0);

	if(live_stream.connected())
	{
		live_stream.send_frame(simulator.current_cycle, kernel_args.framebuffer_width, kernel_args.framebuffer_height, vec_mem.data() + (size_t)kernel_args.framebuffer);

		JsonWriter json;
		json.begin_object()
			.value("cycles", (int64_t)frame_cycles)
			.value("frame_time_s", frame_time)
			.value("wall_time_s", simulation_time)
			.value("power_w", (double)total_power)
			.value("energy_j", (double)total_energy)
			.value("rays", rtcs.empty() ? (uint64_t)kernel_args.framebuffer_size : rtc_log.rays)
			.end_object();
		live_stream.send(LiveStream::MessageType::DONE, json.str());
		live_stream.finish();
	}

	stbi_flip_vertically_on_write(true);
	stbi_write_png("out.png", (int)kernel_args.framebuffer_width, (int)kernel_args.framebuffer_height, 4, vec_mem.data() + (size_t)kernel_args.framebuffer, 0);

    for(auto& tp : tps) delete tp;
    for(auto& sfu : sfus) delete sfu;
    for(auto& l1d : l1ds) delete l1d;
    for(auto& thread_scheduler : thread_schedulers) delete thread_scheduler;
    for(auto& rtc : rtcs) delete rtc;
    for(auto& l3 : l3s) delete l3;
    for(auto& l2 : l2s) delete l2;
    for(auto& dram : drams) delete dram;
}}}

int main(int argc, char* argv[])
{
	Arches::set_full_exe_name(argv[0]);
	Arches::SimulationConfig sim_config(argc, argv);
	Arches::TRaX::run_sim_trax(sim_config);
	return 0;
}