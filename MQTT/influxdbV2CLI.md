# influxdb V2添加配置命令
`influx config create -n xxx -t xxx -u http://localhost:8086 -o xxx -a`
其中-o为organization名称，
-n 为配置名称，
-t为token，
-u为url
-a 为激活为默认配置

# influxdb V2列出所有配置命令
`influx config ls`

# 激活配置
`influx config xxx`

# 列出数据库
`influx bucket ls`

# influx 删除数据库里的某行数据,其中--bucket为数据库名称，--predicate为查询条件，--start为开始时间，--stop为结束时间, 删除前要把要操作的数据库激活为默认配置
***
`influx delete --bucket data --predicate '_measurement="Waste_Daily_Liters"' --start 1970-01-01T00:00:00Z --stop 2024-10-31T00:00:00Z`

