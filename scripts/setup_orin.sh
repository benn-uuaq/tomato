#!/usr/bin/env bash
# 예전 이름. 이제 장비를 보고 역할을 정하는 scripts/setup.sh 를 부른다.
exec bash "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/setup.sh" "$@"
