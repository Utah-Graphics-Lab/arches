#pragma once
#include "vec3.hpp"

#ifndef __riscv
#include "stbi/stb_image.h"
#include "stbi/stb_image_write.h"
#include <vector>
#endif

class Texture2D
{
public:
	union Texel
	{
		uint8_t channel[4];
		uint32_t raw;
	};

	int32_t width;
	int32_t height;
	int32_t comp;
	Texel* texels;

public:
	Texture2D() : texels(nullptr) {};
#ifndef __riscv
	Texture2D(std::string filename)
	{
		printf("Loading: %s\r", filename.c_str());
		stbi_set_flip_vertically_on_load(true);

		stbi_uc* data = stbi_load(filename.c_str(), &width, &height, &comp, 4);
		if(data)
		{
			uint32_t size = width * height;
			texels = (Texel*)malloc(sizeof(Texel) * size);
			for(uint32_t i = 0; i < size; ++i)
				for(uint32_t j = 0; j < 4; ++j)
					texels[i].channel[j] = data[i * 4 + j];

			stbi_image_free(data);
			printf("Loaded: %s \n", filename.c_str());
		}
		else
		{
			texels = nullptr;
			printf("Failed: %s \n", filename.c_str());
		}
	}

	Texture2D(const Texture2D& other)
	{
		memcpy(this, &other, sizeof(Texture2D));
		if (other.texels) {
		uint32_t size = sizeof(Texel) * width * height;
		texels = (Texel*)malloc(size);
		memcpy(texels, other.texels, size);
		}
	}

	Texture2D& operator=(const Texture2D& other)
	{
		if(texels) free(texels);
		memcpy(this, &other, sizeof(Texture2D));
		if (other.texels) {
			uint32_t size = sizeof(Texel) * width * height;
			texels = (Texel*)malloc(size);
			memcpy(texels, other.texels, size);
		}
		return *this;
	}

	~Texture2D()
	{
		if(texels) 
			free(texels);
	}
#endif

	rtm::vec3 sample(const rtm::vec2& uv) const
	{
		if(width == 0 && height == 0) return rtm::vec3(0.0f);
		if(width == 1 && height == 1) return read({0, 0});

		rtm::vec2 _uv = rtm::mod(uv, rtm::vec2(1.0f)) * rtm::vec2(width, height);
		return read_nearest(_uv);

		//rtm::vec2 _uv = rtm::mod(uv, rtm::vec2(1.0f)) * rtm::vec2(width, height);
		//rtm::vec3 s00 = read_nearest(_uv + rtm::vec2(-0.5f, -0.5f));
		//rtm::vec3 s10 = read_nearest(_uv + rtm::vec2(0.5f, -0.5f));
		//rtm::vec3 s01 = read_nearest(_uv + rtm::vec2(-0.5f, 0.5f));
		//rtm::vec3 s11 = read_nearest(_uv + rtm::vec2(0.5f, 0.5f));

		//rtm::vec2 ic = rtm::mod(_uv + rtm::vec2(0.5f), rtm::vec2(1.0f));
		//rtm::vec3 s0 = rtm::mix(s00, s01, ic.y);
		//rtm::vec3 s1 = rtm::mix(s10, s11, ic.y);

		//return rtm::mix(s0, s1, ic.x);
	}

private:
	rtm::vec3 read(const rtm::uvec2& iuv) const
	{
		uint i = iuv[1] * width + iuv[0];
		return rtm::vec3(texels[i].channel[0], texels[i].channel[1], texels[i].channel[2]) * (1.0f / 255.0f);
	}

	rtm::uvec2 get_nearest(const rtm::vec2& uv) const
	{
		rtm::uvec2 mv = rtm::uvec2(width, height);
		rtm::uvec2 iuv = rtm::uvec2(uv[0], uv[1]) + mv;
		iuv[0] = iuv[0] % mv[0]; iuv[1] = iuv[1] % mv[1];
		return  iuv;
	}

	rtm::vec3 read_nearest(const rtm::vec2& uv) const
	{
		rtm::vec3 v = read(get_nearest(uv));
		return v;
	}
};

