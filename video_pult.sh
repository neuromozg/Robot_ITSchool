HOST='192.168.1.184'

gst-launch-1.0 -v \
	rtpbin name=rtpbin latency=200 drop-on-latency=true buffer-mode=4 ntp-time-source=3 ntp-sync=true \
	udpsrc caps="application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)JPEG" port=5000 ! rtpbin.recv_rtp_sink_0 \
	rtpbin. ! rtpjpegdepay ! jpegdec ! autovideosink sync=false \
	udpsrc caps="application/x-rtcp" port=5001 ! rtpbin.recv_rtcp_sink_0 \
	rtpbin.send_rtcp_src_0 ! udpsink port=5005 host=$HOST sync=false async=false
