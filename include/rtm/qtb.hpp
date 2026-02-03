#pragma once

#include "mesh.hpp"

namespace rtm {

//lossless 24 bit quantized triangle block with compressed vertex/index/primID
struct alignas(128) QTB
{
	const static uint BLOCK_SIZE = 128;
	const static uint MAX_TRIS = 16;
	const static uint MAX_VRTS = 16;

	uint32_t : 1;
	uint32_t prim_id0 : 31;
	uint32_t num_vrts : 4;
	uint32_t num_tris : 4;
	uint32_t nx : 5;
	uint32_t nz : 5;
	uint32_t ny : 5;
	uint32_t exp : 8;

	const static uint DATA_SECTOR_SIZE = BLOCK_SIZE - 2 * sizeof(uint32_t);
	BitArray<DATA_SECTOR_SIZE * 8> data;
};
static_assert(sizeof(QTB) == QTB::BLOCK_SIZE);

#ifndef __riscv
inline bool compress(const uint* prim_ids, uint num_tris, const Mesh& mesh, QTB& block)
{
	if(num_tris > QTB::MAX_TRIS) return false;

	uvec3 index_buffer[QTB::MAX_TRIS];
	for(uint i = 0; i < num_tris; ++i)
		index_buffer[i] = mesh.vertex_indices[prim_ids[i]];

	uint num_vrts = 0;
	uvec3 vertex_buffer[QTB::MAX_VRTS];
	std::map<uint, uint> vertex_map;
	for(uint i = 0; i < num_tris; ++i)
	{
		for(uint j = 0; j < 3; ++j)
		{
			uint vertex_index = index_buffer[i][j];
			if(!vertex_map.contains(vertex_index))
			{
				if(num_vrts >= QTB::MAX_VRTS) return false;
				vertex_map[vertex_index] = num_vrts;
				vertex_buffer[num_vrts] = mesh.quantized_vertices[vertex_index];
				num_vrts++;
			}
			index_buffer[i][j] = vertex_map[vertex_index];
		}
	}

	uint mask_x = 0, mask_y = 0, mask_z = 0;
	for(uint i = 0; i < num_vrts; ++i)
	{
		mask_x |= vertex_buffer[i][0] ^ vertex_buffer[0][0];
		mask_y |= vertex_buffer[i][1] ^ vertex_buffer[0][1];
		mask_z |= vertex_buffer[i][2] ^ vertex_buffer[0][2];
	}

	uint max_pfx = 24;
	uint px = min(_lzcnt_u32(mask_x << 8), max_pfx);
	uint py = min(_lzcnt_u32(mask_y << 8), max_pfx);
	uint pz = min(_lzcnt_u32(mask_z << 8), max_pfx);
	uint nx = 24 - px;
	uint ny = 24 - py;
	uint nz = 24 - pz;
	uint ni = 4;

	uint ib_size = ni * 3 * num_tris;
	uint vb_size = (nx + ny + nz) * num_vrts + px + py + pz;
	if(ib_size + vb_size > QTB::DATA_SECTOR_SIZE * 8) return false;

	uint block_ptr = 0;
	for(uint i = 0; i < num_tris; ++i)
	{
		block.data.write(block_ptr, ni, index_buffer[i][0]); block_ptr += ni;
		block.data.write(block_ptr, ni, index_buffer[i][1]); block_ptr += ni;
		block.data.write(block_ptr, ni, index_buffer[i][2]); block_ptr += ni;
	}

	block.data.write(block_ptr, px, vertex_buffer[0].x >> nx); block_ptr += px;
	block.data.write(block_ptr, py, vertex_buffer[0].y >> ny); block_ptr += py;
	block.data.write(block_ptr, pz, vertex_buffer[0].z >> nz); block_ptr += pz;

	for(uint i = 0; i < num_vrts; ++i)
	{
		block.data.write(block_ptr, nx, vertex_buffer[i].x); block_ptr += nx;
		block.data.write(block_ptr, ny, vertex_buffer[i].y); block_ptr += ny;
		block.data.write(block_ptr, nz, vertex_buffer[i].z); block_ptr += nz;
	}

	block.prim_id0 = prim_ids[0];
	block.exp = mesh.exp;
	block.num_tris = num_tris - 1;
	block.num_vrts = num_vrts - 1;
	block.nx = nx;
	block.ny = ny;
	block.nz = nz;

	return true;
}

inline uint decompress(const rtm::QTB& block, uint strip_id, rtm::IntersectionTriangle* tris)
{
	uint prim_id0 = block.prim_id0;
	uint exp = block.exp;
	uint num_tris = block.num_tris + 1;
	uint num_vrts = block.num_vrts + 1;
	uint ni = 4;
	uint nx = block.nx;
	uint ny = block.ny;
	uint nz = block.nz;
	uint px = 24 - nx;
	uint py = 24 - ny;
	uint pz = 24 - nz;

	uint block_ptr = 0;
	uvec3 ib[QTB::MAX_TRIS];
	for(uint i = 0; i < num_tris; ++i)
	{
		ib[i][0] = block.data.read(block_ptr, ni); block_ptr += ni;
		ib[i][1] = block.data.read(block_ptr, ni); block_ptr += ni;
		ib[i][2] = block.data.read(block_ptr, ni); block_ptr += ni;
	}

	uint32_t bgx = (uint64_t)block.data.read(block_ptr, px) << nx; block_ptr += px;
	uint32_t bgy = (uint64_t)block.data.read(block_ptr, py) << ny; block_ptr += py;
	uint32_t bgz = (uint64_t)block.data.read(block_ptr, pz) << nz; block_ptr += pz;

	vec3 vb[QTB::MAX_VRTS];
	for(uint i = 0; i < num_vrts; ++i)
	{
		vb[i].x = i24_to_f32(block.data.read(block_ptr, nx) | bgx, exp); block_ptr += nx;
		vb[i].y = i24_to_f32(block.data.read(block_ptr, ny) | bgy, exp); block_ptr += ny;
		vb[i].z = i24_to_f32(block.data.read(block_ptr, nz) | bgz, exp); block_ptr += nz;
	}

	for(uint i = 0; i < num_tris; ++i)
	{
		tris[i].tri = Triangle(vb[ib[i][0]], vb[ib[i][1]], vb[ib[i][2]]);
		tris[i].id = prim_id0 + i;
	}

	return num_tris;
}
#endif

}