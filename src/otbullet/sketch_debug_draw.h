#pragma once
#include <LinearMath/btIDebugDraw.h>
#include <ot/sketch.h>
#include <ot/logger.h>
#include <ot/glm/glm_types.h>

namespace bt {

	class sketch_debug_draw:public btIDebugDraw
	{
	public:
		sketch_debug_draw();
		~sketch_debug_draw();
		virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) override;
		virtual void setDebugMode(int debugMode) override;
		virtual int	getDebugMode() const  override;
		virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color)  override;
		virtual void	reportErrorWarning(const char* warningString)  override;
		virtual void	draw3dText(const btVector3& location, const char* textString)  override;
        void set_cur_camera_pos(const double3 & cam_pos);

	protected:
		iref<ot::sketch> _sketch;
		iref<ot::logger> _log;
		uint _mode_flags;
        double3 _camera_pos;
        
	};
};//end namespace bt

