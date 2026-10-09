#pragma once
#include <cstdint>
#include <ostream>
#include <stdexcept>
#include <vector>
#include "ftb-wait-diagnostics.hpp"

namespace Arches { namespace Units { namespace TRaX {
// Each RT core owns its records. Only post-execute code reads other units' records.
class MemoryPathOrigins
{
 struct Record { uint64_t token, original, ordinal, address, issue, returned; uint32_t canonical, sector; };
 uint16_t _unit;
 uint64_t _errors{0};
 std::vector<Record> _records;
public:
 struct Totals { uint64_t origins{0}, errors{0}, service_cycles{0}; };
 explicit MemoryPathOrigins(uint16_t unit) : _unit(unit) { if(!unit) throw std::invalid_argument("Memory path unit ID must be nonzero"); }
 uint64_t issue(const FTBWaitDiagnostics::Access& a, uint32_t sector, uint64_t address, uint64_t cycle)
 {
  if(!a.active || sector>=4 || _records.size()+1 >= (1ull<<48)) ++_errors;
  uint64_t token=(uint64_t(_unit)<<48)|(_records.size()+1);
  _records.push_back({token,a.original_ray_id,a.leaf_ordinal,address,cycle,0,a.canonical_ftb_id,sector});
  return token;
 }
 void returned(uint64_t token,uint64_t address,uint64_t cycle)
 {
  const uint64_t index=(token&((1ull<<48)-1));
  if((token>>48)!=_unit || !index || index>_records.size()) { ++_errors; return; }
  auto& r=_records[index-1];
  if(r.token!=token || r.address!=address || r.returned || cycle<r.issue) ++_errors;
  r.returned=cycle;
 }
 static void header(std::ostream& out) { out<<"core,token,original_ray_id,leaf_ordinal,canonical_ftb_id,sector,address,issue,return\n"; }
 Totals write(std::ostream& out,uint32_t core) const
 {
  Totals t; t.errors=_errors;
  for(const auto& r:_records) {
   ++t.origins; t.errors+=!r.returned || r.returned<r.issue;
   if(r.returned>=r.issue) t.service_cycles+=r.returned-r.issue;
   out<<core<<','<<r.token<<','<<r.original<<','<<r.ordinal<<','<<r.canonical<<','<<r.sector<<','<<r.address<<','<<r.issue<<','<<r.returned<<'\n';
  }
  return t;
 }
};
}}}