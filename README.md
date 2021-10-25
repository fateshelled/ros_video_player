# ros_video_player

動画を再生するROS2 Componentsノードです。

## Requirement
- ROS2 Foxy
- OpenCV

## Parameter
- `video_path`: 再生する動画のファイルパス
- `publish_topic_name`: publishするトピック名 (default: "image_raw")
- `image_size`: publishする画像サイズ (default: [640, 480])
- `frame_id`: publishする画像のframe_id (default; "map")
- `loop`: ループ再生するかどうか (default: False)
- `speed`: 再生速度。intではなく、doubleで指定すること。 (default: 1.0)

## DEMO
### Play sample movie.
動画 [ネコ　魚をくわえて歩く](https://www2.nhk.or.jp/archives/creative/material/view.cgi?m=D0002160514_00000)（NHKクリエイティブ・ライブラリー）が2倍速でループ再生されます。
```bash
ros2 launch ros_video_player ros_video_player.launch.py loop:=True speed:=2.0
```

USBカメラ('/dev/video0')を再生します。
```bash
ros2 launch ros_video_player ros_video_player.launch.py video_path:="'0'"
```