#pragma once

namespace bt {

enum EOtFlags {
    OTF_POTENTIAL_TUNNEL_COLLISION = 1,
    OTF_TRANSFORMATION_CHANGED = 2,
    OTF_POTENTIAL_OBJECT_COLLISION = 4,
    OTF_POTENTIAL_TERRAIN_OBJECT_COLLISION = 8,
};

} //namespace bt
