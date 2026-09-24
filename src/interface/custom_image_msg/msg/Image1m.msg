# Uncompressed image with a fixed-capacity payload.
custom_image_msg/Header header
uint32 height
uint32 width
custom_image_msg/String encoding
uint8 is_bigendian
uint32 step
uint8[1000000] data

uint32 DATA_MAX_SIZE=1000000
