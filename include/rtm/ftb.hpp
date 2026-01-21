#pragma once

#include "mesh.hpp"

namespace rtm {

//fp32 triangle block
struct alignas(64) FTB
{
	const static uint BLOCK_SIZE = 64;
	const static uint MAX_TRIS = 2;
	const static uint MAX_VRTS = 9;

	uint32_t is_int   : 1;
	uint32_t prim_idx : 31;
	uint32_t vrt_cnt : 5;
	uint32_t tri_cnt : 5;
	uint32_t nx : 5;
	uint32_t ny : 5;
	uint32_t nz : 5;

	const static uint DATA_SECTOR_SIZE = BLOCK_SIZE - 2 * sizeof(uint32_t);
	BitArray<DATA_SECTOR_SIZE * 8> data;
};
static_assert(sizeof(FTB) == FTB::BLOCK_SIZE);

#ifndef __riscv
inline bool compress(uint prim_idx, uint prim_cnt, const Mesh& mesh, FTB* block = nullptr)
{
	if(prim_cnt > FTB::MAX_TRIS) return false;
	//if(prim_cnt <= 3 && !block) return true;

	uvec3 index_buffer[FTB::MAX_TRIS];
	for(uint i = 0; i < prim_cnt; ++i)
		index_buffer[i] = mesh.vertex_indices[prim_idx + i];

	rtm::vec3 vertex_buffer[FTB::MAX_VRTS];
	uint num_vrts = 0, mask_x = 0, mask_y = 0, mask_z = 0;
	for(uint i = 0; i < prim_cnt; ++i)
	{
		for(uint j = 0; j < 3; ++j)
		{
			uint vertex_index = index_buffer[i][j];
			rtm::vec3 vertex = mesh.vertices[vertex_index];

			uint k;
			for(k = 0; k < num_vrts; ++k)
				if(vertex_buffer[k].x == vertex.x &&
				   vertex_buffer[k].y == vertex.y &&
				   vertex_buffer[k].z == vertex.z)
					break;

			if(k == num_vrts)
			{
				num_vrts++;
				if(num_vrts > FTB::MAX_VRTS) return false;

				vertex_buffer[k] = vertex;
				mask_x |= as_u32(vertex_buffer[k].x) ^ as_u32(vertex_buffer[0].x);
				mask_y |= as_u32(vertex_buffer[k].y) ^ as_u32(vertex_buffer[0].y);
				mask_z |= as_u32(vertex_buffer[k].z) ^ as_u32(vertex_buffer[0].z);
			}

			index_buffer[i][j] = k;
		}
	}

	uint max_pfx = 0;
	uint px = min(_lzcnt_u32(mask_x), max_pfx);
	uint py = min(_lzcnt_u32(mask_y), max_pfx);
	uint pz = min(_lzcnt_u32(mask_z), max_pfx);
	uint nx = 32 - px;
	uint ny = 32 - py;
	uint nz = 32 - pz;
	uint ni = 4;

	uint ib_size = ni * 3 * prim_cnt;
	uint vb_size = (nx + ny + nz) * num_vrts + px + py + pz;
	if(ib_size + vb_size > FTB::DATA_SECTOR_SIZE * 8) return false;
	if(!block) return true;

	uint block_ptr = 0;
	for(uint i = 0; i < prim_cnt; ++i)
	{
		block->data.write(block_ptr, ni, index_buffer[i][0]); block_ptr += ni;
		block->data.write(block_ptr, ni, index_buffer[i][1]); block_ptr += ni;
		block->data.write(block_ptr, ni, index_buffer[i][2]); block_ptr += ni;
	}

	block->data.write(block_ptr, px, as_u32(vertex_buffer[0].x) >> nx); block_ptr += px;
	block->data.write(block_ptr, py, as_u32(vertex_buffer[0].y) >> ny); block_ptr += py;
	block->data.write(block_ptr, pz, as_u32(vertex_buffer[0].z) >> nz); block_ptr += pz;

	for(uint i = 0; i < num_vrts; ++i)
	{
		block->data.write(block_ptr, nx, as_u32(vertex_buffer[i].x)); block_ptr += nx;
		block->data.write(block_ptr, ny, as_u32(vertex_buffer[i].y)); block_ptr += ny;
		block->data.write(block_ptr, nz, as_u32(vertex_buffer[i].z)); block_ptr += nz;
	}

	block->prim_idx = prim_idx;
	block->tri_cnt = prim_cnt - 1;
	block->vrt_cnt = num_vrts - 1;
	block->nx = nx - 1;
	block->ny = ny - 1;
	block->nz = nz - 1;

	return true;
}

inline uint decompress(const rtm::FTB& block, rtm::IntersectionTriangle* tris)
{
	uint prim_idx = block.prim_idx;
	uint tri_cnt = block.tri_cnt + 1;
	uint vrt_cnt = block.vrt_cnt + 1;
	uint ni = 4;
	uint nx = block.nx + 1;
	uint ny = block.ny + 1;
	uint nz = block.nz + 1;
	uint px = 32 - nx;
	uint py = 32 - ny;
	uint pz = 32 - nz;

	uint block_ptr = 0;
	uvec3 ib[FTB::MAX_TRIS];
	for(uint i = 0; i < tri_cnt; ++i)
	{
		ib[i][0] = block.data.read(block_ptr, ni); block_ptr += ni;
		ib[i][1] = block.data.read(block_ptr, ni); block_ptr += ni;
		ib[i][2] = block.data.read(block_ptr, ni); block_ptr += ni;
	}

	uint32_t bgx = (int64_t)block.data.read(block_ptr, px) << nx; block_ptr += px;
	uint32_t bgy = (int64_t)block.data.read(block_ptr, py) << ny; block_ptr += py;
	uint32_t bgz = (int64_t)block.data.read(block_ptr, pz) << nz; block_ptr += pz;

	rtm::vec3 vb[FTB::MAX_VRTS];
	for(uint i = 0; i < vrt_cnt; ++i)
	{
		vb[i].x = as_f32((block.data.read(block_ptr, nx)) | bgx); block_ptr += nx;
		vb[i].y = as_f32((block.data.read(block_ptr, ny)) | bgy); block_ptr += ny;
		vb[i].z = as_f32((block.data.read(block_ptr, nz)) | bgz); block_ptr += nz;
	}

	for(uint i = 0; i < tri_cnt; ++i)
	{
		tris[i].tri = Triangle(vb[ib[i][0]], vb[ib[i][1]], vb[ib[i][2]]);
		tris[i].id = prim_idx + i;
	}

	return tri_cnt;
}
#endif

}