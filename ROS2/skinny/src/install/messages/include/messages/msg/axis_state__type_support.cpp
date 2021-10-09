// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from messages:msg/AxisState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "messages/msg/axis_state__struct.hpp"
#include "rosidl_generator_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"
#include "string"
#include "vector"

namespace messages {

	namespace msg {

		namespace rosidl_typesupport_introspection_cpp {

			void AxisState_init_function(
				void* message_memory,
				rosidl_generator_cpp::MessageInitialization _init) {
				new(message_memory) messages::msg::AxisState(_init);
			}

			void AxisState_fini_function(void* message_memory) {
				auto typed_message = static_cast<messages::msg::AxisState*>(message_memory);
				typed_message->~AxisState();
			}

			static const ::rosidl_typesupport_introspection_cpp::MessageMember AxisState_message_member_array[3] = {
				{
					"joystick",												// name
					::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8, // type
					0,														// upper bound of string
					nullptr,												// members of sub message
					false,													// is array
					0,														// array size
					false,													// is upper bound
					offsetof(messages::msg::AxisState, joystick),			// bytes offset in struct
					nullptr,												// default value
					nullptr,												// size() function pointer
					nullptr,												// get_const(index) function pointer
					nullptr,												// get(index) function pointer
					nullptr													// resize(index) function pointer
				},
				{
					"axis",													// name
					::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8, // type
					0,														// upper bound of string
					nullptr,												// members of sub message
					false,													// is array
					0,														// array size
					false,													// is upper bound
					offsetof(messages::msg::AxisState, axis),				// bytes offset in struct
					nullptr,												// default value
					nullptr,												// size() function pointer
					nullptr,												// get_const(index) function pointer
					nullptr,												// get(index) function pointer
					nullptr													// resize(index) function pointer
				},
				{
					"state",												// name
					::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT, // type
					0,														// upper bound of string
					nullptr,												// members of sub message
					false,													// is array
					0,														// array size
					false,													// is upper bound
					offsetof(messages::msg::AxisState, state),				// bytes offset in struct
					nullptr,												// default value
					nullptr,												// size() function pointer
					nullptr,												// get_const(index) function pointer
					nullptr,												// get(index) function pointer
					nullptr													// resize(index) function pointer
				}};

			static const ::rosidl_typesupport_introspection_cpp::MessageMembers AxisState_message_members = {
				"messages::msg", // message namespace
				"AxisState",	 // message name
				3,				 // number of fields
				sizeof(messages::msg::AxisState),
				AxisState_message_member_array, // message members
				AxisState_init_function,		// function to initialize message memory (memory has to be allocated)
				AxisState_fini_function			// function to terminate message instance (will not free memory)
			};

			static const rosidl_message_type_support_t AxisState_message_type_support_handle = {
				::rosidl_typesupport_introspection_cpp::typesupport_identifier,
				&AxisState_message_members,
				get_message_typesupport_handle_function,
			};

		} // namespace rosidl_typesupport_introspection_cpp

	} // namespace msg

} // namespace messages

namespace rosidl_typesupport_introspection_cpp {

	template <>
	ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC const rosidl_message_type_support_t*
	get_message_type_support_handle<messages::msg::AxisState>() {
		return &::messages::msg::rosidl_typesupport_introspection_cpp::AxisState_message_type_support_handle;
	}

} // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C" {
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t*
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, messages, msg, AxisState)() {
	return &::messages::msg::rosidl_typesupport_introspection_cpp::AxisState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
