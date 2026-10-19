from iimt_api import *

print("尝试登录...")
login_result,handle = API_Login()
print(f"登录后的句柄值: {handle.value}")
print(f"API_Login 返回值: {login_result}")
if login_result:
    print("登录成功")
else:
    print("登录失败")
if login_result:
    print("开始获取变位机位置")
    result,positioner_position = API_GetPositionerPosition(handle)
    if result:
        print(f"变位机位置: {[positioner_position.description[i] for i in range(6)]}")
    else:
        print("获取变位机位置失败")
    
