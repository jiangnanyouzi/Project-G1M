#pragma once

#ifndef G1MG_
#define G1MG_

#include "G1MG/G1MGMaterial.h"
#include "G1MG/G1MGVertexBuffer.h"
#include "G1MG/G1MGVertexAttributes.h"
#include "G1MG/G1MGIndexBuffer.h"
#include "G1MG/G1MGBonePalette.h"
#include "G1MG/G1MGSubMesh.h"
#include "G1MG/G1MGMesh.h"

#define SECTION1_MAGIC   0x00010001
#define MATERIALS_MAGIC  0x00010002
#define SECTION3_MAGIC  0x00010003
#define VERTEX_BUFFERS_MAGIC  0x00010004
#define VERTEX_ATTRIBUTES_MAGIC  0x00010005
#define JOINT_PALETTES_MAGIC  0x00010006
#define INDEX_BUFFER_MAGIC  0x00010007
#define SUBMESH_MAGIC  0x00010008
#define MESH_MAGIC  0x00010009
#define MESHLET_TRIANGLES_MAGIC 0x0001000b
#define MESHLET_INFO_MAGIC      0x0001000c
#define MESHLET_BVH_MAGIC       0x0001000e
#define MESHLET_MAP_MAGIC       0x00010010


template <bool bBigEndian>
struct G1MGSubSectionHeader
{
	uint32_t magic;
	uint32_t size;
	uint32_t count;

	G1MGSubSectionHeader(G1MGSubSectionHeader* ptr) : G1MGSubSectionHeader(*ptr)
	{
		if (bBigEndian)
		{
			LITTLE_BIG_SWAP(magic);
			LITTLE_BIG_SWAP(size);
			LITTLE_BIG_SWAP(count);
		}
	}
};

template <bool bBigEndian>
struct G1MGHeader
{
	uint32_t platform;
	uint32_t reserved;
	float min_x;
	float min_y;
	float min_z;
	float max_x;
	float max_y;
	float max_z;
	uint32_t sectionCount;

	G1MGHeader(G1MGHeader* ptr) : G1MGHeader(*ptr)
	{
		if (bBigEndian)
		{
			LITTLE_BIG_SWAP(platform);
			LITTLE_BIG_SWAP(min_x);
			LITTLE_BIG_SWAP(min_y);
			LITTLE_BIG_SWAP(min_z);
			LITTLE_BIG_SWAP(max_x);
			LITTLE_BIG_SWAP(max_y);
			LITTLE_BIG_SWAP(max_z);
			LITTLE_BIG_SWAP(sectionCount);
		}
	}
};

template <bool bBigEndian>
struct G1MG
{
	uint32_t version = 0;
	float min_x = 0, min_y = 0, min_z = 0;
	float max_x = 0, max_y = 0, max_z = 0;

	std::vector<G1MGMaterial<bBigEndian>> materials;
	std::vector<G1MGVertexBuffer<bBigEndian>> vertexBuffers;
	std::vector<G1MGVertexAttributeSet<bBigEndian>> vertexAttributeSets;
	std::vector<G1MGJointPalette<bBigEndian>> jointPalettes;
	std::vector<G1MGIndexBuffer<bBigEndian>> indexBuffers;
	std::vector<G1MGSubmesh<bBigEndian>> submeshes;
	std::vector<G1MGMeshGroup<bBigEndian>> meshGroups;

	// Nioh 3 meshlet sections (0x1000b / 0x1000c / 0x1000e / 0x10010)
	std::vector<uint32_t> meshletTriangles; // 0x1000b packed 10-bit triangles
	std::vector<uint32_t> meshletInfoOffsets; // 0x1000c: offset into 0x1000b
	std::vector<uint32_t> meshletInfoCounts;  // 0x1000c: count of triangles
	std::vector<uint32_t> meshletInfoU1;      // 0x1000c: remap table start in IB
	std::vector<uint32_t> meshletInfoU2;      // 0x1000c: remap table length
	std::vector<bool> meshletIsLeaf;          // 0x1000e BVH leaf flags
	std::vector<uint16_t> meshletToSubmesh;   // 0x10010 mapping

	G1MG(BYTE* buffer, uint32_t startOffset, G1MS<bBigEndian>* internalSkel, G1MS<bBigEndian>* externalSkel, std::map<uint32_t, uint32_t>* globalToFinal)
	{
		uint32_t offset = startOffset;
		//Read headers
		GResourceHeader sectionHeader = reinterpret_cast<GResourceHeader<bBigEndian>*>(buffer + offset);
		version = sectionHeader.chunkVersion;
		offset = startOffset + 12;
		G1MGHeader<bBigEndian> g1mgHeader = reinterpret_cast<G1MGHeader<bBigEndian>*>(buffer + offset);
		min_x = g1mgHeader.min_x;
		min_y = g1mgHeader.min_y;
		min_z = g1mgHeader.min_z;
		max_x = g1mgHeader.max_x;
		max_y = g1mgHeader.max_y;
		max_z = g1mgHeader.max_z;
		offset += sizeof(G1MGHeader<bBigEndian>);
		uint32_t checkpoint = offset;
		for (auto i = 0; i < g1mgHeader.sectionCount; i++)
		{
			offset = checkpoint;
			G1MGSubSectionHeader<bBigEndian> subSectionHeader = reinterpret_cast<G1MGSubSectionHeader<bBigEndian>*>(buffer + offset);
			offset += sizeof(G1MGSubSectionHeader<bBigEndian>);
			switch (subSectionHeader.magic)
			{
			case SECTION1_MAGIC:
				break;
			case MATERIALS_MAGIC:
				for (auto j = 0; j < subSectionHeader.count; j++)
				{
					materials.push_back(G1MGMaterial<bBigEndian>(buffer, offset));
				}
				break;
			case SECTION3_MAGIC:
				break;
			case VERTEX_BUFFERS_MAGIC:
			{
				/*for (auto j = 0; j < subSectionHeader.count; j++)
				{
					vertexBuffers.push_back(G1MGVertexBuffer<bBigEndian>(buffer, offset,sectionHeader.chunkVersion));
				}*/
				int totalCount = 0;
				int check;
				int accOffsetBuffer = 0;
				while (totalCount != subSectionHeader.count)
				{
					uint32_t peek = *(uint32_t*)(buffer + offset + 4);
					if (peek != 1)
					{
						vertexBuffers.push_back(G1MGVertexBuffer<bBigEndian>(buffer, offset, sectionHeader.chunkVersion));
						totalCount += 1;
						if (sectionHeader.chunkVersion >= 0x30303435 || vertexBuffers[vertexBuffers.size() - 1].unk2 != 0)
						{
							while (true)
							{
								check = *(int*)(buffer + offset);
								if (check != 0x80000000)
									break;
								int stride = *(int*)(buffer + offset + 4);
								int count = *(int*)(buffer + offset + 8);
								vertexBuffers.push_back(G1MGVertexBuffer<bBigEndian>(vertexBuffers[0].bufferAdress, accOffsetBuffer, stride, count));
								accOffsetBuffer += stride * count;
								totalCount += 1;
								offset += 0x10;
							}
						}

					}
					else
					{
						vertexBuffers.push_back(G1MGVertexBuffer<bBigEndian>(buffer, offset, sectionHeader.chunkVersion));
						totalCount += 1;

						while (true)
						{
							check = *(int*)(buffer + offset);
							if (check != 0x80000000)
								break;
							int stride = *(int*)(buffer + offset + 4);
							int count = *(int*)(buffer + offset + 8);
							vertexBuffers.push_back(G1MGVertexBuffer<bBigEndian>(vertexBuffers[0].bufferAdress, accOffsetBuffer, stride, count));
							accOffsetBuffer += stride * count;
							totalCount += 1;
							offset += 0x10;
						}
					}
				}
				break;
			}

			case VERTEX_ATTRIBUTES_MAGIC:
				for (auto j = 0; j < subSectionHeader.count; j++)
				{
					vertexAttributeSets.push_back(G1MGVertexAttributeSet<bBigEndian>(buffer, offset));
				}
				break;
			case JOINT_PALETTES_MAGIC:
				for (auto j = 0; j < subSectionHeader.count; j++)
				{
					jointPalettes.push_back(G1MGJointPalette<bBigEndian>(buffer, offset,internalSkel,externalSkel,globalToFinal));
				}
				break;
			case INDEX_BUFFER_MAGIC:
				for (auto j = 0; j < subSectionHeader.count; j++)
				{
					indexBuffers.push_back(G1MGIndexBuffer<bBigEndian>(buffer, offset, sectionHeader.chunkVersion));
				}
				break;
			case SUBMESH_MAGIC:
				for (auto j = 0; j < subSectionHeader.count; j++)
				{
					G1MGSubmesh<bBigEndian> subM = reinterpret_cast<G1MGSubmesh<bBigEndian>*>(buffer + offset);
					submeshes.push_back(std::move(subM));
					offset += sizeof(G1MGSubmesh<bBigEndian>);
				}
				break;
			case MESH_MAGIC:
				for (auto j = 0; j < subSectionHeader.count; j++)
				{
					meshGroups.push_back(G1MGMeshGroup<bBigEndian>(buffer, offset, sectionHeader.chunkVersion));
				}
				break;
			case MESHLET_TRIANGLES_MAGIC:
			{
				uint32_t b_count = *(uint32_t*)(buffer + offset);
				uint32_t b_stride = *(uint32_t*)(buffer + offset + 4);
				if (bBigEndian) { LITTLE_BIG_SWAP(b_count); LITTLE_BIG_SWAP(b_stride); }
				meshletTriangles.resize(b_count);
				memcpy(meshletTriangles.data(), buffer + offset + 8, sizeof(uint32_t) * b_count);
				if (bBigEndian)
					for (auto& v : meshletTriangles) LITTLE_BIG_SWAP(v);
				break;
			}
			case MESHLET_INFO_MAGIC:
			{
				uint32_t c_count = *(uint32_t*)(buffer + offset);
				uint32_t c_stride = *(uint32_t*)(buffer + offset + 4);
				if (bBigEndian) { LITTLE_BIG_SWAP(c_count); LITTLE_BIG_SWAP(c_stride); }
				meshletInfoOffsets.resize(c_count);
				meshletInfoCounts.resize(c_count);
				meshletInfoU1.resize(c_count);
				meshletInfoU2.resize(c_count);
				for (uint32_t j = 0; j < c_count; j++)
				{
					uint32_t off = offset + 8 + j * c_stride;
					uint32_t vals[4] = { 0 };
					uint32_t copyLen = c_stride < 16 ? c_stride : 16;
					memcpy(vals, buffer + off, copyLen);
					if (bBigEndian)
						for (int k = 0; k < 4; k++) LITTLE_BIG_SWAP(vals[k]);
					meshletInfoOffsets[j] = vals[0];
					meshletInfoCounts[j] = vals[1];
					meshletInfoU1[j] = vals[2];
					meshletInfoU2[j] = vals[3];
				}
				break;
			}
			case MESHLET_BVH_MAGIC:
			{
				uint32_t e_count = *(uint32_t*)(buffer + offset);
				uint32_t e_stride = *(uint32_t*)(buffer + offset + 4);
				if (bBigEndian) { LITTLE_BIG_SWAP(e_count); LITTLE_BIG_SWAP(e_stride); }
				meshletIsLeaf.resize(e_count, true);
				if (e_count == meshletInfoOffsets.size() && e_stride >= 24)
				{
					for (uint32_t j = 0; j < e_count; j++)
					{
						uint32_t off = offset + 8 + j * e_stride;
						int32_t child0 = *(int32_t*)(buffer + off + 16);
						int32_t child1 = *(int32_t*)(buffer + off + 20);
						if (bBigEndian) { LITTLE_BIG_SWAP(child0); LITTLE_BIG_SWAP(child1); }
						meshletIsLeaf[j] = (child0 == -1 && child1 == -1);
					}
				}
				break;
			}
			case MESHLET_MAP_MAGIC:
			{
				uint32_t m_count = *(uint32_t*)(buffer + offset);
				uint32_t m_stride = *(uint32_t*)(buffer + offset + 4);
				if (bBigEndian) { LITTLE_BIG_SWAP(m_count); LITTLE_BIG_SWAP(m_stride); }
				meshletToSubmesh.resize(m_count, 0xFFFF);
				if (m_count == meshletInfoOffsets.size() && m_stride >= 12)
				{
					for (uint32_t j = 0; j < m_count; j++)
					{
						uint32_t off = offset + 8 + j * m_stride;
						uint32_t c_val = *(uint32_t*)(buffer + off + 8);
						if (bBigEndian) LITTLE_BIG_SWAP(c_val);
						meshletToSubmesh[j] = (c_val >> 16) & 0xFFFF;
					}
				}
				break;
			}
			default:
				break;
			}
			checkpoint += subSectionHeader.size;
		}
	}
};

#endif //!G1MG_