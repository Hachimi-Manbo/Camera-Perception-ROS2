import cv2

def split_video_into_frames(video_path, frames_folder):
    # 使用OpenCV打开视频文件
    cap = cv2.VideoCapture(video_path)

    # 检查视频是否成功打开
    if not cap.isOpened():
        print("Error: 无法打开视频文件.")
        return

    # 获取视频的帧率
    fps = int(cap.get(cv2.CAP_PROP_FPS))

    # 初始化帧计数器
    count = 0

    # 读取视频中的每一帧
    while True:
        # 读取下一帧
        ret, frame = cap.read()

        # 如果读取帧失败，则退出循环
        if not ret:
            break

        # 构建帧的文件名
        frame_filename = f"{frames_folder}/frame_{count:04d}.jpg"

        # 保存帧
        cv2.imwrite(frame_filename, frame)

        # 更新帧计数器
        count += 1

    # 释放视频捕获对象
    cap.release()

    print(f"视频已成功拆分为 {count} 帧，并保存到 {frames_folder}.")

# 使用示例
video_path = '/home/jetson/Documents/CV/ros2_project/ros2_cam/road_video.mp4'  # 替换为您的视频文件路径
frames_folder = './'  # 替换为用于保存帧的文件夹路径
split_video_into_frames(video_path, frames_folder)
