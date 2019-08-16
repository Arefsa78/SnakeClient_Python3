#### start.sh
```bash
./start.sh {type} {team_name}
# or
./start.sh {team_name}
```
> types: best, greedy, random, hand, your

#### Your client
> change YourClient.py in get_action function \
function should return one of **l, r, u, d** directions \
run your code with `./start.sh your (my_team)` \
**your** is type of the team (default is **best**) \
**my_team** is team name that is optional

#### other clients
##### random
> **random** client (type=**random**) moves randomly
##### greedy
> **greedy** client (type=**greedy**) moves towards target point without considering walls
##### best
> **best** client (type=**best**) moves towards target with considering walls




