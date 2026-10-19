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
    print("尝试设置关机延迟时间...")
    delay_time = 22
    result = API_SetPowerOffDelayTime(handle,delay_time)
    if result:
        print("设置关机延迟时间成功")
    else:
        print("设置关机延迟时间失败")
    
