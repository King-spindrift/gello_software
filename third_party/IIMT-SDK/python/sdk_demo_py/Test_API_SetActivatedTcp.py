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
        print("尝试设置激活TCP...")
        tcp_name=b"TCP_2"
        set_activated_tcp_result = API_SetActivatedTcp(handle,tcp_name)
        print(f"API_SetActivatedTcp 返回值：{set_activated_tcp_result}")
        if set_activated_tcp_result:
            print("设置激活TCP成功...")
        else:
            print("设置激活TCP失败...")

