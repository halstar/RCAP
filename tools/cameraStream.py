# Web streaming example
# Source code from the official PiCamera package
# http://picamera.readthedocs.io/en/latest/recipes2.html#web-streaming

import io
import cv2
import shutil
import logging
import socketserver
from http import server

PAGE="""\
<html>
<head>
</head>
<body>
<center><img src="stream.jpg" width="480" height="320"></center>
</body>
</html>
"""

class StreamingOutput(object):
    def __init__(self):
        pass

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.jpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    ret, frame = camera.read()
                    cv2.imwrite('image.jpg', frame)
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.end_headers()
                    with open('image.jpg', 'rb') as content:
                        shutil.copyfileobj(content, self.wfile)
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

# open camera
camera = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)

# set dimensions
camera.set(cv2.CAP_PROP_FRAME_WIDTH , 480)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)

output = StreamingOutput()

try:
    address = ('', 8000)
    server = StreamingServer(address, StreamingHandler)
    server.serve_forever()
finally:
    camera.release()
