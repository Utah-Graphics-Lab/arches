#include <vector>

#include "ramulator2/src/base/base.h"
#include "ramulator2/src/dram/dram.h"
#include "ramulator2/src/addr_mapper/addr_mapper.h"
#include "ramulator2/src/memory_system/memory_system.h"

namespace Ramulator {

// Base for Arches' custom address mappers. Ramulator's IAddrMapper interface
// (addr_mapper.h) ships no shared base for the standard "linear" setup() — upstream
// keeps that in linear_mappers.cpp with no header, and the submodule is read-only.
// So we implement the little setup() our mappers need: derive each level's address-bit
// width and the transaction offset from the DRAM organization.
class ArchesLinearMapperBase : public IAddrMapper {
public:
    IDRAM* m_dram = nullptr;

    int m_num_levels = -1;
    std::vector<int> m_addr_bits;
    Addr_t m_tx_offset = -1;

protected:
    void setup(IFrontEnd*, IMemorySystem* memory_system) {
        m_dram = memory_system->get_ifce<IDRAM>();

        const auto& count = m_dram->m_organization.count;
        m_num_levels = static_cast<int>(count.size());
        m_addr_bits.resize(m_num_levels);
        for (size_t level = 0; level < m_addr_bits.size(); level++) {
            m_addr_bits[level] = calc_log2(count[level]);
        }

        m_addr_bits[m_num_levels - 1] -= calc_log2(m_dram->m_internal_prefetch_size);

        int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
        m_tx_offset = calc_log2(tx_bytes);
    }
};

class RoRaBaChCo final : public ArchesLinearMapperBase, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, RoRaBaChCo, "RoRaBaChCo", "Applies a RoRaBaChCo mapping to the address.");

public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
        ArchesLinearMapperBase::setup(frontend, memory_system);
    }

    void apply(Request& req) override {
        req.addr_vec.resize(m_num_levels, -1);
        Addr_t addr = req.addr >> m_tx_offset;
        req.addr_vec[m_addr_bits.size() - 1] = slice_lower_bits(addr, m_addr_bits[m_addr_bits.size() - 1]);
        req.addr_vec[0] = slice_lower_bits(addr, m_addr_bits[0]);
        req.addr_vec[2] = slice_lower_bits(addr, m_addr_bits[2]);
        req.addr_vec[1] = slice_lower_bits(addr, m_addr_bits[1]);
        req.addr_vec[3] = slice_lower_bits(addr, m_addr_bits[3]);
    }
};

class RoBgBaRaChCo final : public ArchesLinearMapperBase, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, RoBgBaRaChCo, "RoBgBaRaChCo", "Applies a RoBgBaRaChCo mapping to the address.");

public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
        ArchesLinearMapperBase::setup(frontend, memory_system);
    }

    void apply(Request& req) override {
        req.addr_vec.resize(m_num_levels, -1);
        Addr_t addr = req.addr >> m_tx_offset;
        req.addr_vec[m_addr_bits.size() - 1] = slice_lower_bits(addr, m_addr_bits[m_addr_bits.size() - 1]);
        req.addr_vec[0] = slice_lower_bits(addr, m_addr_bits[0]);
        req.addr_vec[1] = slice_lower_bits(addr, m_addr_bits[1]);
        req.addr_vec[3] = slice_lower_bits(addr, m_addr_bits[3]);
        req.addr_vec[2] = slice_lower_bits(addr, m_addr_bits[2]);
        req.addr_vec[4] = slice_lower_bits(addr, m_addr_bits[4]);
    }
};

}   // namespace Ramulator
