#!/bin/bash
# Проверка работоспособности pocket_web: рукопожатие WebSocket на /ws (ответ 101) и признак
# работающего приёмника "run":1 в приветствии hello. Корень при пустом каталоге -html отвечает 404,
# поэтому на него проверка не опирается. Только встроенные средства bash: пакеты не добавляются.
exec 3<>/dev/tcp/127.0.0.1/8081 || exit 1
printf 'GET /ws HTTP/1.1\r\nHost: 127.0.0.1:8081\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n' >&3

# Приветствие приходит сразу за ответом 101; читается побайтно не дольше 2 с
reply=''
deadline=$((SECONDS + 2))
while ((SECONDS <= deadline)); do
   IFS= read -r -N 1 -t 1 byte <&3 || break
   reply+=$byte
   case $reply in
      'HTTP/1.1 101 '*'"run":1'*) exit 0 ;;
      'HTTP/1.1 101 '*'"run":0'*) exit 1 ;;
   esac
done
exit 1
