DEVICE='/dev/video0'
CAPS='image/jpeg,width=640,height=480,framerate=30/1'
HOST='192.168.1.76'

gst-launch-1.0 -v \
	rtpbin name=rtpbin latency=200 drop-on-latency=true buffer-mode=4 ntp-time-source=3 ntp-sync=true rtcp-sync-send-time=false  \
	v4l2src device=$DEVICE ! $CAPS ! jpegparse ! rtpjpegpay ! queue ! rtpbin.send_rtp_sink_0  \
	rtpbin.send_rtp_src_0 ! udpsink port=5000 host=$HOST sync=true async=false \
	rtpbin.send_rtcp_src_0 ! udpsink port=5001 host=$HOST sync=false async=false \
	udpsrc port=5005 caps="application/x-rtcp" ! rtpbin.recv_rtcp_sink_0
