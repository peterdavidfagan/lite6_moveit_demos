install(PROGRAMS
  src/motion_planning_python_api_tutorial.py
  src/ps4_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
	DIRECTORY config launch
	DESTINATION share/${PROJECT_NAME}
)
