`ngrok http --domain=upward-daring-macaque.ngrok-free.app 1880`
 `ngrok start --all`
`ngrok config edit `


打开多个端口的方法：
https://www.youtube.com/watch?v=js1lxR12hHo&list=PLT9nYbazZFLm57rP0o5qtmw8lUmJgPA5Y&index=12
1. ngrok config add-authtoken $YOUR_AUTHTOKEN
2. sudo nano .config/ngrok/config.yml   
3. add:
`tunnels:
 http:
    addr: 1880
    proto: http

http:
    addr: 3000
    proto: http

1. ngrok start --all