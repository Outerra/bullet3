#pragma once
#ifndef __OTBULLET_SHAPE_INFO_CFG_H__
#define __OTBULLET_SHAPE_INFO_CFG_H__

struct shape_info_base {
    const unsigned short _mesh_id;
    shape_info_base(unsigned short mesh_id)
        : _mesh_id(mesh_id)
    {}
};

#endif // __OTBULLET_SHAPE_INFO_CFG_H__