#ifndef __ROBOT_ACTION_H__
#define __ROBOT_ACTION_H__

namespace cstem {
class RobotAction {
	public:
		RobotAction() {}
		~RobotAction() {}

		void start() {
			_isRunning = true;
		}

		void stop() {
			_isRunning = false;
		}
		
		bool isRunning() {
			return _isRunning;
		}

	private:
		bool _isRunning;
};
}

#endif
