#include "driftless/alliance/SkillsAlliance.hpp"

namespace driftless {
namespace alliance {
EAlliance SkillsAlliance::getAlliance() { return EAlliance::NONE; }

std::string SkillsAlliance::getName() { return "SKILLS"; }
}  // namespace alliance
}  // namespace driftless