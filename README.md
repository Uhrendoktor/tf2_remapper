# tf2_remapper
A ROS2-Node for remapping frame_ids in generic Message types on a given list of topics: i.e. the `/tf` or `/tf_static` topics.

Currently frame-ids in the following message parts will be remapped:\
`std_msgs::msg::Header` (frame_id)\
`tf2_msgs::msg::TFMessage` (frame_id, child_frame_id)

## Usage
Arguments of the `tf2_remapper` ROS2-Node:
`frame_mapping` - `string` (json):
```json
[
    { "reg": "a|b|c", "rep": "[$&]"}, // albert likes apples -> [a]l[b]ert likes [a]pples
    ...
    { "in": "wierd_base_link", "out": "base_link"},
    { "in": "wierd_base_link/other_link", "out": "base_link/other_link"}, // only replaces full occurences
    ...
]
```
`topic_mapping` - `string` (json) same rules as in `frame_mapping`:
```json
[
    { "reg": "abc", "rep": "def"}, // /abc/test -> /def/test
    ...
    { "in": "/velodyne_points", "out": "/reframed_points"},
    // only replaces full occurences
    ...
]
```

In booth settings only the first match will be used!

`topic_refresh_rate` - `int` (`default: 1000ms`)
the intervall in which the available topics are refreshed. When a subscribed topic goes down, the remapper will follow by closing its subscription and publisher. When a new valid topic is discovered the remapper will open a new subscription and publisher.
Changing this on runtime is not possible!