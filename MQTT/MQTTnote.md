node-red端口：1880
mosquitto端口：1883
grafana端口：3000
influxdb端口：8086

grafana重设密码命令
***
`sudo grafana-cli admin reset-admin-password admin`

influxdb重设密码命令
***
`influx -username admin -password admin`

运行influx
***
`sudo influx`




mosquitto配置文件路径
***
`/etc/mosquitto/mosquitto.conf`

mosquitto查看所有话题
***
`mosquitto_sub -v -t '#'`

mosquitto发布消息
***
`mosquitto_pub -t "topic" -m "message"`



mosquitto查看所有订阅者
***
`mosquitto_sub -v -t '$SYS/#' | grep -E 'clients/connected/|clients/disconnected/'`



node-red配置文件路径
***
`/home/pi/.node-red/settings.js`

grafana配置文件路径
***
`/etc/grafana/grafana.ini`

influxdb配置文件路径
***
`/etc/influxdb/influxdb.conf`

influxdb数据库创建
***
`CREATE DATABASE "mqtt"`

influxdb数据库删除
***
`DROP DATABASE "mqtt"`

influxdb数据库查看
***
`SHOW DATABASES`

influxdb数据库使用
***
`USE "mqtt"`

influxdb数据表创建
***
`CREATE RETENTION POLICY "mqtt_retention" ON "mqtt" DURATION 30d REPLICATION 1 DEFAULT`

influxdb数据表查看
***
`SHOW RETENTION POLICIES ON "mqtt"`

influxdb数据表删除
***
`DROP RETENTION POLICY "mqtt_retention" ON "mqtt"`

influxdb数据表修改
***
`ALTER RETENTION POLICY "mqtt_retention" ON "mqtt" DURATION 30d REPLICATION 1 DEFAULT`

