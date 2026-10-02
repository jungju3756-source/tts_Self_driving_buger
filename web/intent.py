# -*- coding: utf-8 -*-
"""
intent.py  —  말 → 의도 (장소 이름 찾기 · 정지 감지). 두 음성 엔진이 같이 쓴다.

- resolve_place(): 모델/음성인식이 준 장소 문자열을 등록된 장소로 확정 (못 하면 None — 추측 안 함)
- is_stop():       '멈춰/정지/스톱/그만' — LLM 을 거치지 않는 비상 정지용
- parse_command(): Gemini 없이 쓰는 간단한 규칙 기반 해석 (브라우저 음성 인식 모드)

RobotNav 앱의 resolveLiveDestination 구조를 따르되 한글 그룹명 버그(화장실→화장)는 없앴다:
숫자 접미사만 떼고 한글은 절대 자르지 않는다.
"""

import re

STOP_WORDS = ('멈춰', '멈춤', '멈추', '정지', '스톱', '스탑', 'stop', '그만', '서')
STOP_EXACT_ONLY = ('서',)          # '서' 는 한 글자라 단독 발화일 때만

# 음성 인식이 숫자를 한글로 줄 때: '일번' '이 번' '삼번' '첫 번째'
KO_NUM = {'일': 1, '이': 2, '삼': 3, '사': 4, '오': 5, '육': 6, '칠': 7, '팔': 8, '구': 9, '십': 10,
          '하나': 1, '둘': 2, '셋': 3, '넷': 4, '다섯': 5,
          '첫': 1, '두': 2, '세': 3, '네': 4}
GO_HINTS = ('가', '이동', '데려', '안내', '보내', '출발', '가줘', '가자', '갈래')
LIST_HINTS = ('어디어디', '목록', '장소', '뭐 있', '뭐있', '어디 갈 수', '어디갈수')
WHERE_HINTS = ('지금 어디', '어디야', '현재 위치', '어디에 있')


def norm(s):
    return re.sub(r'\s+', '', s or '').lower()


def _ko_numbers(s):
    """'일번' '이 번째' → '1번'"""
    def rep(m):
        return f'{KO_NUM[m.group(1)]}번'
    keys = '|'.join(sorted(KO_NUM, key=len, reverse=True))
    return re.sub(rf'({keys})\s*번(째)?(?![에엔])', rep, s)   # '이번에' 는 숫자 아님


def is_stop(text):
    t = norm(text)
    if not t:
        return False
    for w in STOP_WORDS:
        if w in STOP_EXACT_ONLY:
            if t in (w, w + '.', w + '!'):
                return True
        elif w in t:
            return True
    return False


def resolve_place(raw, places):
    """raw(모델/인식 결과) → 등록된 장소 이름 또는 None.
    places: {이름: {'aliases': [...]}, ...}
    """
    if not raw or len(raw) > 40 or '<' in raw or '>' in raw:   # 이상한 문자열은 시도도 안 함
        return None
    q = norm(_ko_numbers(raw))
    # 1) 이름·별칭 정확히 일치
    for name, p in places.items():
        if q == norm(name) or q in [norm(a) for a in p.get('aliases') or []]:
            return name
    # 2) 서로 포함 관계 — 딱 하나만 걸릴 때만 인정
    hits = set()
    for name, p in places.items():
        for cand in [name] + list(p.get('aliases') or []):
            c = norm(cand)
            if c and (c in q or q in c):
                hits.add(name)
    if len(hits) == 1:
        return hits.pop()
    return None


def find_place_in_sentence(text, places):
    """문장 안에서 장소 이름/별칭을 찾는다. 가장 긴 것 우선, 서로 다른 장소가 둘 이상이면 None."""
    t = norm(_ko_numbers(text))
    found = []
    for name, p in places.items():
        for cand in [name] + list(p.get('aliases') or []):
            c = norm(cand)
            if c and c in t:
                found.append((len(c), name))
    if not found:
        return None, []
    found.sort(reverse=True)
    names = sorted({n for _, n in found})
    if len(names) > 1:
        # '10번' 안에 '1번' 이 들어가는 경우처럼 짧은 쪽이 긴 쪽에 포함되면 긴 쪽을 택한다
        longest = found[0][1]
        lc = norm(longest)
        others = [n for n in names if n != longest and norm(n) not in lc]
        if others:
            return None, names
        return longest, names
    return names[0], names


def parse_command(text, places):
    """규칙 기반 해석 → (action, place, reply)
    action: 'stop' | 'go' | 'list' | 'where' | 'ask' | 'unknown'
    """
    if is_stop(text):
        return 'stop', None, '멈출게요.'
    t = norm(text)
    place, cands = find_place_in_sentence(text, places)
    if place:
        # '정수기 어디야' 처럼 위치만 묻는 말은 바로 출발하지 않는다 (RobotNav 규칙)
        if '어디' in t and not any(h in t for h in GO_HINTS[1:]):
            return 'ask', None, f'{place}(으)로 안내할까요? "{place} 가줘" 라고 말해 주세요.'
        return 'go', place, f'{place}(으)로 갈게요.'
    if cands:
        return 'ask', None, f"{', '.join(cands)} 중 어디로 갈까요?"
    if any(norm(h) in t for h in WHERE_HINTS):
        return 'where', None, None
    if any(norm(h) in t for h in LIST_HINTS):
        names = ', '.join(places) or '없어요'
        return 'list', None, f'갈 수 있는 곳은 {names} 이에요.'
    if any(h in t for h in GO_HINTS):
        return 'ask', None, '어디로 갈까요? ' + (', '.join(places) + ' 중에서 말해 주세요.' if places else '')
    return 'unknown', None, '잘 못 알아들었어요. 장소 이름을 말해 주세요.'
