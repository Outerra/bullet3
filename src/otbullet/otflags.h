#pragma once

namespace bt {

enum EOtFlags {
    OTF_POTENTIAL_TUNNEL_COLLISION =                1 << 0,
    OTF_TRANSFORMATION_CHANGED =                    1 << 1,
    OTF_POTENTIAL_OBJECT_COLLISION =                1 << 2,
    OTF_POTENTIAL_TERRAIN_OBJECT_COLLISION =        1 << 3,
    OTF_TERRAIN_EXCLUDER =                          1 << 4,
    OTF_SENSOR_GHOST_OBJECT =                       1 << 5,
    OTF_DISABLE_OT_WORLD_COLLISIONS =               1 << 6
};

} //namespace bt
