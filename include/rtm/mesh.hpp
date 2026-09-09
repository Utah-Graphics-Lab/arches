#pragma once

#include "vec2.hpp"
#include "vec3.hpp"
#include "uvec3.hpp"
#include "triangle.hpp"
#include "texture.hpp"

namespace rtm {

struct Material
{
	enum
	{
		EMISSIVE,
		LAMBERTIAN,
		MICROFACET,
	};

	uint8_t model{LAMBERTIAN};
	uint8_t use_nm : 1;
	uint8_t use_am : 1;
	uint8_t use_rm : 1;
	uint8_t use_mm : 1;
	Texture2D albedo_texture;
	rtm::vec3 albedo;
	float roughness;
	float metalness;
	float ior;
};

}

#ifndef __riscv
#include <vector>
#include <string>
#include <fstream>
#include <cassert>
#include <map>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <deque>

namespace rtm {

class Mesh
{
public:
	std::vector<rtm::uvec3> vertex_indices;
	std::vector<rtm::uvec3> normal_indices;
	std::vector<rtm::uvec3> tex_coord_indices;

	std::vector<rtm::vec3> vertices;
	std::vector<rtm::vec3> normals;
	std::vector<rtm::vec2> tex_coords;
	std::vector<uvec3> quantized_vertices;
	uint8_t exp; //bias exponent

	std::vector<uint> material_indices;
	std::vector<std::string> material_names;
	std::string mtl_lib;
	
	std::vector<Texture2D> textures;
	std::vector<Material> materials;

public:
	Mesh(std::string file_path) : mtl_lib("")
	{
 		if(!load_obj(file_path.c_str()))
		{
			printf("Mesh: failed to load mesh OBJ file %s\n", file_path.c_str());
			return;
		}
		if(mtl_lib.length() > 0)
		{
			std::string parent_folder = parent_path(file_path);
			if (!load_mtl(swap_path(mtl_lib, parent_folder), parent_folder + "textures/"))
			{
				printf("Mesh: failed to load mesh textures or MTL file %s\n", mtl_lib.c_str());
				return;
			}
		}
	}

	static inline rtm::vec2 read_vec2(char* line)
	{
		rtm::vec2 v;
		uint index = 0;
		for(uint i = 0; i < 2; ++i)
		{
			uint start = index;
			while(line[index] != ' ' && line[index] != '\0') index++;
			char c = line[index];
			line[index] = '\0';
			v[i] = std::atof(&line[start]);

			if(c == '\0') break; //can't consume endline outside of consume line by convetion
			index++;
		}

		return v;
	}

	static inline rtm::vec3 read_vec3(char* line)
	{
		rtm::vec3 v;
		uint index = 0;
		for(uint i = 0; i < 3; ++i)
		{
			uint start = index;
			while(line[index] != ' ' && line[index] != '\0') index++;
			char c = line[index];
			line[index] = '\0';
			v[i] = std::atof(&line[start]);

			if(c == '\0') break;
			index++;
		}

		return v;
	}

	static inline uint read_uint(char* data, size_t& index)
	{
		uint v;

		uint start = index;
		while(data[index] != ' ' && data[index] != '\n') index++;
		char c = data[index];
		data[index] = '\0';
		v = std::atoi(&data[start]);
		data[index] = c;

		return v;
	}

	static inline float read_float(char* data, size_t& index)
	{
		float v;

		uint start = index;
		while(data[index] != ' ' && data[index] != '\n') index++;
		char c = data[index];
		data[index] = '\0';
		v = std::atof(&data[start]);
		data[index] = c;

		return v;
	}

	static inline std::string read_str(char* data, size_t& index)
	{
		std::string str = "";
		while(data[index] != '\n')
		{
			str += data[index];
			index++;
		}
		if(str.back() == '\r') str.pop_back();
		return str;
	}

	static inline std::string read_str(char* data)
	{
		std::string str = data;
		while(str.back() == '\n' || str.back() == '\r') str.pop_back();
		return str;
	}

	static inline void consume_line(char* data, size_t& index)
	{
		while(data[index] != '\n') index++;
		index++; //consume endline
	}

	static inline void read_face(char* line, rtm::uvec3& vrt_inds, rtm::uvec3& txcd_inds, rtm::uvec3& nrml_inds)
	{
		uint index = 0;
		for(uint i = 0; i < 3; ++i)
		{
			uint start = index;
			while(line[index] != '/' && line[index] != ' ' && line[index] != '\0') ++index;
			char c = line[index];
			line[index] = '\0';
			vrt_inds[i] = std::atoi(&line[start]) - 1;

			if(c == '\0') break;
			index++;
			if(c == ' ') continue;

			start = index;
			while(line[index] != '/' && line[index] != ' ' && line[index] != '\0') index++;
			c = line[index];
			line[index] = '\0';
			uint txcd_ind = std::atoi(&line[start]);
			if(txcd_ind != 0) txcd_inds[i] = txcd_ind - 1;

			if(c == '\0') break;
			index++;
			if(c == ' ') continue;

			start = index;
			while(line[index] != ' ' && line[index] != '\0') index++;
			c = line[index];
			line[index] = '\0';
			uint nrml_ind = std::atoi(&line[start]);
			if(nrml_ind != 0) nrml_inds[i] = nrml_ind - 1;

			if(c == '\0') break; //can't consume endline outside of consume line by convetion
			index++;
		}
	}


	bool load_obj(const char* file_path)
	{
		printf("Mesh: Loading: %s\n", file_path);

		std::ifstream is(file_path);
		if(!is.is_open()) return false;
		is.seekg(0, std::ios_base::end);
		std::size_t size = is.tellg();
		is.seekg(0, std::ios_base::beg);

		std::vector<char> data(size + 2);
		is.read((char*)&data[0], size);
		data[size] = '\n';
		data[size+1] = '\0';
		is.close();

		//simple hashes for switch statement
		constexpr uint8_t v = 'v' + ' ';

		constexpr uint8_t vt = 'v' + 't';
		constexpr uint8_t vn = 'v' + 'n';
		constexpr uint8_t vp = 'v' + 'p';

		constexpr uint8_t f = 'f' + ' ';
		constexpr uint8_t l = 'l' + ' ';

		constexpr uint8_t usemtl = 'u' + 's';
		constexpr uint8_t mtllib = 'm' + 't';

		constexpr uint8_t o = 'o' + ' ';
		constexpr uint8_t g = 'g' + ' ';

		constexpr uint8_t s = 's' + ' ';

		uint64_t next_line_start = 0;
		uint64_t line_number = 0;

		char c = data[next_line_start];
		while(next_line_start < size)
		{
			data[next_line_start] = c;

			//get the next line
			uint64_t line_size = 0;
			char* line = &data[next_line_start];
			while(line[line_size++] != '\n');
			next_line_start += line_size;

			//insert null charter after newline
			c = data[next_line_start];
			data[next_line_start] = '\0';

			//ignore comments and empty lines
			if(line[0] != 'v' &&
				line[0] != 'f' &&
				line[0] != 'l' &&
				line[0] != 'u' &&
				line[0] != 'm' &&
				line[0] != 'o' &&
				line[0] != 'g' &&
				line[0] != 's'
				) continue;

			//simple hash of first two characters on the line
			uint8_t type = line[0] + line[1];

			//advance to start of data
			uint data_start_index = 0;
			while(line[data_start_index] != ' ' && data_start_index < line_size) data_start_index++;
			while(line[data_start_index] == ' ' && data_start_index < line_size) data_start_index++;
			if(data_start_index == line_size) continue;

			switch(type)
			{
			case v:
				vertices.push_back(read_vec3(line + data_start_index));
				break;

			case vt:
				tex_coords.push_back(read_vec2(line + data_start_index));
				break;

			case vn:
				normals.push_back(rtm::normalize(read_vec3(line + data_start_index)));
				break;

			case f:
				vertex_indices.emplace_back(0);
				tex_coord_indices.emplace_back(~0x0u);
				normal_indices.emplace_back(~0x0u);
				material_indices.emplace_back(material_names.empty() ? 0u : (uint)(material_names.size() - 1u));
				read_face(line + data_start_index, vertex_indices.back(), tex_coord_indices.back(), normal_indices.back());
				break;

			case mtllib:
				mtl_lib = read_str(line + data_start_index);
				break;

			case usemtl:
				material_names.push_back(read_str(line + data_start_index));
				break;

			case vp: //na
			case l: //na
			case o: //na
			case g: //na
			case s: //na
				break;

			default:
				printf("\nMesh: Invalid line: %jd\n", line_number);
				break;
			}

			line_number++;
		}

		tex_coords.emplace_back(0.0f, 0.0f);
		for(uint i = 0; i < tex_coord_indices.size(); ++i)
		{
			if(tex_coord_indices[i][0] == ~0x0u)
			{
				tex_coord_indices[i][0] = tex_coord_indices[i][1] = tex_coord_indices[i][2] = tex_coords.size() - 1;
			}
		}

		for(uint i = 0; i < normal_indices.size(); ++i)
		{
			if(normal_indices[i][0] == ~0x0u)
			{
				rtm::vec3 gn = rtm::normalize(rtm::cross(vertices[vertex_indices[i][1]] - vertices[vertex_indices[i][0]], vertices[vertex_indices[i][2]] - vertices[vertex_indices[i][0]]));
				normal_indices[i][0] = normal_indices[i][1] = normal_indices[i][2] = normals.size();
				normals.push_back(gn);
			}
		}

		printf("Mesh: Size: %.1f MiB\n", ((float)sizeof(rtm::vec3) * vertices.size() + (float)sizeof(rtm::uvec3) * vertex_indices.size()) / (1 << 20));
		printf("Mesh: Triangles: %d\n", vertex_indices.size());
		printf("Mesh: Vertices: %d\n", vertices.size());

		return true;
	}

	static inline std::string parent_path(std::string path)
	{
		int i = 0;
		for (i = path.size() - 2; i >= 0; --i)
			if (path.at(i) == '/')
				break;
		return path.substr(0, i + 1);
	}

	static inline std::string swap_path(std::string current_path, std::string new_path)
	{
		int i = 0;
		for(i = current_path.size() - 2; i >= 0; --i)
			if(current_path.at(i) == '/')
				break;
		return new_path + current_path.substr(i + 1);
	}

	static inline std::string to_string(rtm::vec3 v)
	{
		return std::to_string(v[0]) + " " + std::to_string(v[1]) + " " + std::to_string(v[2]);
	}

	bool load_mtl(std::string file_path, std::string texture_path)
	{
		printf("Loading: %s\r", file_path.c_str());

		std::ifstream is(file_path);
		if(!is.is_open()) return false;

		is.seekg(0, std::ios_base::end);
		std::size_t size = is.tellg();
		is.seekg(0, std::ios_base::beg);

		std::vector<char> vec(size);
		is.read((char*)&vec[0], size);
		vec.push_back('\n');
		is.close();

		char* data = vec.data();
		size = vec.size() - 1;

		//simple hashes for switch statement
		constexpr uint8_t newmtl = 'n' + 'e';

		constexpr uint8_t Ns = 'N' + 's';
		constexpr uint8_t Ka = 'K' + 'a';
		constexpr uint8_t Kd = 'K' + 'd';
		constexpr uint8_t Ks = 'K' + 's';
		constexpr uint8_t Ke = 'K' + 'e';
		constexpr uint8_t Ni = 'N' + 'i';

		constexpr uint8_t d = 'd' + ' ';
		constexpr uint8_t illum = 'i' + 'l';

		constexpr uint8_t map_x = 'm' + 'a';
		constexpr uint8_t refl = 'r' + 'e';

		std::unordered_map<std::string, uint> texture_index_map;
		std::unordered_map<std::string, uint> material_index_map;

		size_t index = 0;
		size_t line_number = 0;
		while(index < size)
		{
			if(data[index] == '\0')
				break;

			if(data[index] == '\n')
			{
				index++;
				line_number++;
				continue;
			}

			if(data[index] == '#')
			{
				consume_line(data, index);
				line_number++;
				continue;
			}

			//simple hash of first two characters on the line
			uint8_t type = data[index] + data[index + 1];

			//advance to start of data
			while(data[index] != ' ') index++;
			while(data[index] == ' ') index++;

			switch(type)
			{
			case newmtl:
			{
				material_index_map[read_str(data, index)] = materials.size();
				materials.emplace_back();
				break;
			}

			case Ns:
			{
				float ns = read_vec3(data)[0];
				float roughness = 1.0f - sqrtf(ns / 1000.0f);
				if(!materials.back().use_rm)
					materials.back().roughness = roughness;
				break;
			}

			case Kd:
			{
				rtm::vec3 kd = read_vec3(data);
				if(!materials.back().use_am)
					materials.back().albedo = kd;
				break;
			}

			case Ke:
			{
				rtm::vec3 ke = read_vec3(data);
				if(rtm::length2(ke) > 0.0f)
				{
					materials.back().model = Material::EMISSIVE;
					materials.back().albedo = ke;
				}
				break;
			}

			case illum: //na
			{
				uint model = read_uint(data, index);
				if(materials.back().model == Material::EMISSIVE) break;
				switch(model)
				{
				case 1: //diffuse
					materials.back().model = Material::LAMBERTIAN;
					break;
				case 2: //dielectric
					materials.back().model = Material::MICROFACET;
					if(!materials.back().use_mm)
						materials.back().metalness = 0.0f;
					break;
				case 3: //metalic
					materials.back().model = Material::MICROFACET;
					if(!materials.back().use_mm)
						materials.back().metalness = 1.0f;
					break;
				default:
					materials.back().model = Material::LAMBERTIAN;
					break;
				}
				break;
			}

			case map_x: //na
			{
				if(data[index - 2] == 'd') //map_Kd
				{
					std::string str = swap_path(read_str(data, index), texture_path);
					materials.back().albedo_texture = Texture2D(str);
					materials.back().use_am = 1;
				}
				//else if(data[index - 2] == 's') //map_Ns
				//{
				//	std::string str = swap_path(read_str(data, index), texture_path);
				//	materials.back().texture.add(LDRTexture(str), 3);
				//	materials.back().use_rm = 1;
				//}
				//else if(data[index - 2] == 'l') //map_refl
				//{
				//	std::string str = swap_path(read_str(data, index), texture_path);
				//	materials.back().texture.add(LDRTexture(str), 7);
				//	materials.back().use_mm = 1;
				//}
				//else if(data[index - 2] == 'p') //map_Bump
				//{
				//	if(data[index] == '-')
				//	{
				//		index += 4;
				//		while(data[index++] != ' ');
				//	}

				//	std::string str = swap_path(read_str(data, index), texture_path);
				//	materials.back().texture.add(LDRTexture(str), 4);
				//	materials.back().use_nm = 1;
				//}
				break;
			}

		#if 0
			case refl:
			{
				std::string str = swap_path(read_str(data, index), texture_path);
				if(texture_index_map.find(str) == texture_index_map.end())
				{
					texture_index_map[str] = textures.size();
					textures.emplace_back(str);
				}
				materials.back().metalness_index = texture_index_map[str];
				break;
			}
		#endif


			case Ks: //na
			case Ka: //na
			case Ni: //na
			case d:  //na
				break;

			default:
				printf("\nInvalid line: %lld\n", line_number);
				break;
			}

			consume_line(data, index);
			line_number++;
		}

		std::vector<uint> material_reassignment_map(material_names.size());
		for(uint i = 0; i < material_names.size(); ++i)
		{
			std::string name = material_names[i];
			uint new_index = material_index_map[name];
			material_reassignment_map[i] = new_index;
		}

		for(uint i = 0; i < material_indices.size(); ++i)
			material_indices[i] = material_reassignment_map[material_indices[i]];

		printf("Loaded: %s \n", file_path.c_str());

		return true;
	}

	uint size() const { return (uint)vertex_indices.size(); }

	Triangle get_triangle(uint32_t index) const
	{
		return {vertices[vertex_indices[index][0]], vertices[vertex_indices[index][1]], vertices[vertex_indices[index][2]]};
	}

	void get_triangles(std::vector<Triangle>& triangles) const
	{
		triangles.clear();
		for(uint32_t i = 0; i < vertex_indices.size(); ++i)
			triangles.emplace_back(get_triangle(i));
	}

	uint8_t quantize_verts()
	{
		float32_bf max(0.0f);
		for(auto& v : vertices)
			for(uint i = 0; i < 3; ++i)
				max.f32 = rtm::max(std::abs(v[i]), max.f32);

		max.mantisa = 0;
		max.exp++;
		exp = max.exp;

		for(auto& v : vertices)
		{
			quantized_vertices.push_back(0);
			for(uint i = 0; i < 3; ++i)
			{
				quantized_vertices.back()[i] = f32_to_i24(v[i], exp);
				v[i] = i24_to_f32(quantized_vertices.back()[i], exp);
			}
		}
		printf("Mesh: Quantized with exp: %f\n", rtm::float32_bf(0, exp, 0).f32);

		return exp;
	}
};


class MeshGraph
{
public:
	//build face graph
	std::vector<rtm::uvec3> face_graph;
	MeshGraph(const Mesh& mesh)
	{
		face_graph.resize(mesh.vertex_indices.size(), uvec3(~0u));

		std::map<std::pair<uint32_t, uint32_t>, std::pair<uint32_t, uint32_t>> edge_to_face_map;
		for(uint f = 0; f < mesh.vertex_indices.size(); ++f)
		{
			uvec3 face = mesh.vertex_indices[f];
			for(uint e = 0; e < 3; ++e)
			{
				std::pair<uint32_t, uint32_t> edge(face[(e + 1) % 3], face[(e + 2) % 3]);
				if(edge.first > edge.second) std::swap(edge.first, edge.second);
				if(edge_to_face_map.find(edge) != edge_to_face_map.end())
				{
					//link
					uint32_t other_f = edge_to_face_map[edge].first;
					uint32_t other_e = edge_to_face_map[edge].second;
					face_graph[f][e] = other_f;
					face_graph[other_f][other_e] = f;
				}
				else edge_to_face_map[edge] = {f, e};
			}
		}

		for(auto& face : face_graph)
		{
			if(face[0] == face[1] && face[1] == face[2])
			{
				face[0] = ~0u;
				face[1] = ~0u;
				face[2] = ~0u;
			}
			if(face[0] == face[1])
			{
				face[0] = ~0u;
				face[1] = ~0u;
			}
			if(face[1] == face[2])
			{
				face[1] = ~0u;
				face[2] = ~0u;
			}
			if(face[2] == face[0])
			{
				face[2] = ~0u;
				face[0] = ~0u;
			}
		}
	}
};

}

#endif