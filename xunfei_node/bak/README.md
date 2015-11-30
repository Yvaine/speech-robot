log
recognize
tk1从麦克风中把音频信号传输给我的笔记本，然后将信号传给ifly,ifly返回识别结果
可以正确识别，只是识别速度比较慢，在线识别和传回的时间比较长，每识别出一个
命令开启一路会话, 阻塞接收， 从说话开始计时，3s获得识别的结果

recvsound.cpp 我的电脑端,windows系统，tcp 通讯， 接收音频
sendsound.cpp tk1端 , ubuntu系统， tcp通讯， 从显卡中读音频，发送音频

cmd
从识别的结果解析，把命令代码发送给tk1，

