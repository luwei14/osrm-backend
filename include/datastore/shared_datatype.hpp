#ifndef SHARED_DATA_TYPE_HPP
#define SHARED_DATA_TYPE_HPP

#include "util/osrm_exception.hpp"
#include "util/simple_logger.hpp"

#include <cstdint>

#include <array>

namespace osrm
{
namespace datastore
{

// Added at the start and end of each block as sanity check
const constexpr char CANARY[] = "OSRM";

struct SharedDataLayout
{
    enum BlockID
    {
        NAME_OFFSETS = 0,
        NAME_BLOCKS,
        NAME_CHAR_LIST,
        NAME_ID_LIST,
        VIA_NODE_LIST,
        GRAPH_NODE_LIST,
        GRAPH_EDGE_LIST,
        COORDINATE_LIST,
        TURN_INSTRUCTION,
        TRAVEL_MODE,
        R_SEARCH_TREE,
        GEOMETRIES_INDEX,
        GEOMETRIES_LIST,
        GEOMETRIES_INDICATORS,
        HSGR_CHECKSUM,
        TIMESTAMP,
        FILE_INDEX_PATH,
        CORE_MARKER,
        NUM_BLOCKS
    };

    std::array<uint64_t, NUM_BLOCKS> num_entries;
    std::array<uint64_t, NUM_BLOCKS> entry_size;

    SharedDataLayout() : num_entries(), entry_size() {}

    void PrintInformation() const
    {
        util::SimpleLogger().Write(logDEBUG) << "NAME_OFFSETS         "
                                             << ": " << GetBlockSize(NAME_OFFSETS);
        util::SimpleLogger().Write(logDEBUG) << "NAME_BLOCKS          "
                                             << ": " << GetBlockSize(NAME_BLOCKS);
        util::SimpleLogger().Write(logDEBUG) << "NAME_CHAR_LIST       "
                                             << ": " << GetBlockSize(NAME_CHAR_LIST);
        util::SimpleLogger().Write(logDEBUG) << "NAME_ID_LIST         "
                                             << ": " << GetBlockSize(NAME_ID_LIST);
        util::SimpleLogger().Write(logDEBUG) << "VIA_NODE_LIST        "
                                             << ": " << GetBlockSize(VIA_NODE_LIST);
        util::SimpleLogger().Write(logDEBUG) << "GRAPH_NODE_LIST      "
                                             << ": " << GetBlockSize(GRAPH_NODE_LIST);
        util::SimpleLogger().Write(logDEBUG) << "GRAPH_EDGE_LIST      "
                                             << ": " << GetBlockSize(GRAPH_EDGE_LIST);
        util::SimpleLogger().Write(logDEBUG) << "COORDINATE_LIST      "
                                             << ": " << GetBlockSize(COORDINATE_LIST);
        util::SimpleLogger().Write(logDEBUG) << "TURN_INSTRUCTION     "
                                             << ": " << GetBlockSize(TURN_INSTRUCTION);
        util::SimpleLogger().Write(logDEBUG) << "TRAVEL_MODE          "
                                             << ": " << GetBlockSize(TRAVEL_MODE);
        util::SimpleLogger().Write(logDEBUG) << "R_SEARCH_TREE        "
                                             << ": " << GetBlockSize(R_SEARCH_TREE);
        util::SimpleLogger().Write(logDEBUG) << "GEOMETRIES_INDEX     "
                                             << ": " << GetBlockSize(GEOMETRIES_INDEX);
        util::SimpleLogger().Write(logDEBUG) << "GEOMETRIES_LIST      "
                                             << ": " << GetBlockSize(GEOMETRIES_LIST);
        util::SimpleLogger().Write(logDEBUG) << "GEOMETRIES_INDICATORS"
                                             << ": " << GetBlockSize(GEOMETRIES_INDICATORS);
        util::SimpleLogger().Write(logDEBUG) << "HSGR_CHECKSUM        "
                                             << ": " << GetBlockSize(HSGR_CHECKSUM);
        util::SimpleLogger().Write(logDEBUG) << "TIMESTAMP            "
                                             << ": " << GetBlockSize(TIMESTAMP);
        util::SimpleLogger().Write(logDEBUG) << "FILE_INDEX_PATH      "
                                             << ": " << GetBlockSize(FILE_INDEX_PATH);
        util::SimpleLogger().Write(logDEBUG) << "CORE_MARKER          "
                                             << ": " << GetBlockSize(CORE_MARKER);
    }

    template <typename T> inline void SetBlockSize(BlockID bid, uint64_t entries)
    {
        num_entries[bid] = entries;
        entry_size[bid] = sizeof(T);
    }

    inline uint64_t GetBlockSize(BlockID bid) const
    {
        // special bit encoding
        if (bid == GEOMETRIES_INDICATORS || bid == CORE_MARKER)
        {
            return (num_entries[bid] / 32 + 1) * entry_size[bid];
        }

        return num_entries[bid] * entry_size[bid];
    }

    inline uint64_t GetSizeOfLayout() const
    {
        return GetBlockOffset(NUM_BLOCKS) + NUM_BLOCKS * 2 * sizeof(CANARY);
    }

    inline uint64_t GetBlockOffset(BlockID bid) const
    {
        uint64_t result = sizeof(CANARY);
        for (auto i = 0; i < bid; i++)
        {
            result += GetBlockSize((BlockID)i) + 2 * sizeof(CANARY);
        }
        return result;
    }

    template <typename T, bool WRITE_CANARY = false>
    inline T *GetBlockPtr(char *shared_memory, BlockID bid)
    {
        T *ptr = (T *)(shared_memory + GetBlockOffset(bid));
        if (WRITE_CANARY)
        {
            char *start_canary_ptr = shared_memory + GetBlockOffset(bid) - sizeof(CANARY);
            char *end_canary_ptr = shared_memory + GetBlockOffset(bid) + GetBlockSize(bid);
            std::copy(CANARY, CANARY + sizeof(CANARY), start_canary_ptr);
            std::copy(CANARY, CANARY + sizeof(CANARY), end_canary_ptr);
        }
        else
        {
            char *start_canary_ptr = shared_memory + GetBlockOffset(bid) - sizeof(CANARY);
            char *end_canary_ptr = shared_memory + GetBlockOffset(bid) + GetBlockSize(bid);
            bool start_canary_alive = std::equal(CANARY, CANARY + sizeof(CANARY), start_canary_ptr);
            bool end_canary_alive = std::equal(CANARY, CANARY + sizeof(CANARY), end_canary_ptr);
            if (!start_canary_alive)
            {
                throw util::exception("Start canary of block corrupted.");
            }
            if (!end_canary_alive)
            {
                throw util::exception("End canary of block corrupted.");
            }
        }

        return ptr;
    }
};

enum SharedDataType
{
    CURRENT_REGIONS,
    LAYOUT_1,
    DATA_1,
    LAYOUT_2,
    DATA_2,
    LAYOUT_NONE,
    DATA_NONE
};

struct SharedDataTimestamp
{
    SharedDataType layout;
    SharedDataType data;
    unsigned timestamp;
};
}
}

#endif /* SHARED_DATA_TYPE_HPP */
