#include "sketch_debug_draw.h"

namespace bt {

	sketch_debug_draw::sketch_debug_draw()
	{
		_sketch = ot::sketch::get();
		_log = ot::logger::get();
		_mode_flags = 0;
        _camera_pos = double3(0.0);
	}


	sketch_debug_draw::~sketch_debug_draw()
	{
	}

	void sketch_debug_draw::drawLine(const btVector3 & from, const btVector3 & to, const btVector3 & color)
	{
        const float3 f = float3(double3(from.x(), from.y(), from.z()) - _camera_pos);
        const float3 t = float3(double3(to.x(), to.y(), to.z()) - _camera_pos);
        _sketch->set_color((uint(color.x() * 255)) | (uint(color.y() * 255)) << 8 | (uint(color.z() * 255)) << 16 | 0xff000000);
		_sketch->draw_line(f, true);
		_sketch->draw_line(t, false);
	}

	void sketch_debug_draw::setDebugMode(int debugMode)
	{
		_mode_flags = debugMode;
	}

	int sketch_debug_draw::getDebugMode() const
	{
		return _mode_flags;
	}

	void sketch_debug_draw::drawContactPoint(const btVector3 & PointOnB, const btVector3 & normalOnB, btScalar distance, int lifeTime, const btVector3 & color)
	{
		drawLine(PointOnB, PointOnB - normalOnB, color);
	}

	void sketch_debug_draw::reportErrorWarning(const char * warningString)
	{
		_log->warning_log(warningString);
	}

	void sketch_debug_draw::draw3dText(const btVector3 & location, const char * textString)
	{
		_log->info_log("draw3dText!");
	}

    void sketch_debug_draw::set_cur_camera_pos(const double3 & cam_pos)
    {
        _camera_pos = cam_pos;
        _sketch->set_position(cam_pos);
    }
} //end namespace bt