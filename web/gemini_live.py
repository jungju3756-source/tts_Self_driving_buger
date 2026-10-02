# -*- coding: utf-8 -*-
"""
gemini_live.py  —  Gemini Live API (BidiGenerateContent) WebSocket 클라이언트 (tornado)

    브라우저 마이크 16 kHz PCM ─► send_audio() ─► Gemini
    Gemini ─► on_audio(24 kHz PCM bytes)   스피커로
           ─► on_event({'type': 'input_text' | 'output_text' | 'turn_complete' | 'interrupted' ...})
           ─► on_tool(name, args) → dict   (await 가능) → toolResponse 로 회신

STT·LLM·TTS 를 Live 세션 하나로 처리한다 (캡스톤 결정 A안).
SDK 대신 원시 프로토콜을 쓴다 — Pi 에 pip 가 없고, 이전 RobotNav 앱도 같은 방식.
"""

import asyncio
import base64
import json
import logging

from tornado.httpclient import HTTPRequest
from tornado.websocket import websocket_connect

log = logging.getLogger('gemini')

ENDPOINT = ('wss://generativelanguage.googleapis.com/ws/'
            'google.ai.generativelanguage.v1beta.GenerativeService.BidiGenerateContent')


class GeminiLive:

    def __init__(self, api_key, model, voice, system_text, tools,
                 on_audio, on_event, on_tool, silence_ms=600):
        self.api_key = api_key
        self.model = model if model.startswith('models/') else f'models/{model}'
        self.voice = voice
        self.system_text = system_text
        self.tools = tools
        self.on_audio, self.on_event, self.on_tool = on_audio, on_event, on_tool
        self.silence_ms = silence_ms
        self.ws = None
        self.ready = asyncio.Event()
        self.closed = False
        self._reader = None

    async def start(self, timeout=15):
        req = HTTPRequest(f'{ENDPOINT}?key={self.api_key}', connect_timeout=10, request_timeout=10)
        self.ws = await websocket_connect(req, max_message_size=16 * 1024 * 1024)
        setup = {
            'model': self.model,
            'generationConfig': {
                'responseModalities': ['AUDIO'],
                'speechConfig': {
                    'voiceConfig': {'prebuiltVoiceConfig': {'voiceName': self.voice}},
                    'languageCode': 'ko-KR',
                },
            },
            'systemInstruction': {'parts': [{'text': self.system_text}]},
            'tools': [{'functionDeclarations': self.tools}],
            'inputAudioTranscription': {},
            'outputAudioTranscription': {},
            'realtimeInputConfig': {
                'automaticActivityDetection': {'silenceDurationMs': self.silence_ms},
            },
        }
        await self.ws.write_message(json.dumps({'setup': setup}))
        self._reader = asyncio.ensure_future(self._read_loop())
        await asyncio.wait_for(self.ready.wait(), timeout)

    async def _read_loop(self):
        try:
            while True:
                raw = await self.ws.read_message()
                if raw is None:
                    break
                msg = json.loads(raw if isinstance(raw, str) else raw.decode('utf-8'))
                await self._handle(msg)
        except Exception as e:                       # 세션 끊김은 상위에 이벤트로 알림
            log.warning('read loop: %s', e)
            self.on_event({'type': 'error', 'text': str(e)})
        finally:
            reason = getattr(self.ws, 'close_reason', None) if self.ws else None
            code = getattr(self.ws, 'close_code', None) if self.ws else None
            self.closed = True
            self.on_event({'type': 'closed', 'code': code, 'reason': reason})

    async def _handle(self, msg):
        if 'setupComplete' in msg:
            self.ready.set()
            self.on_event({'type': 'ready'})
            return
        sc = msg.get('serverContent')
        if sc:
            if sc.get('interrupted'):
                self.on_event({'type': 'interrupted'})
            it = (sc.get('inputTranscription') or {}).get('text')
            if it:
                self.on_event({'type': 'input_text', 'text': it})
            ot = (sc.get('outputTranscription') or {}).get('text')
            if ot:
                self.on_event({'type': 'output_text', 'text': ot})
            for part in (sc.get('modelTurn') or {}).get('parts', []):
                data = (part.get('inlineData') or {}).get('data')
                if data:
                    self.on_audio(base64.b64decode(data))
            if sc.get('turnComplete'):
                self.on_event({'type': 'turn_complete'})
            return
        tc = msg.get('toolCall')
        if tc:
            responses = []
            for fc in tc.get('functionCalls', []):
                name, args = fc.get('name'), fc.get('args') or {}
                self.on_event({'type': 'tool_call', 'name': name, 'args': args})
                try:
                    result = self.on_tool(name, args)
                    if asyncio.iscoroutine(result):
                        result = await result
                except Exception as e:
                    result = {'ok': False, 'error': str(e)}
                responses.append({'id': fc.get('id'), 'name': name, 'response': result})
            await self.ws.write_message(json.dumps({'toolResponse': {'functionResponses': responses}}))
            return
        if 'goAway' in msg:
            self.on_event({'type': 'go_away', 'detail': msg['goAway']})

    async def send_audio(self, pcm16):
        if self.closed or not self.ready.is_set():
            return
        await self.ws.write_message(json.dumps({'realtimeInput': {'audio': {
            'data': base64.b64encode(pcm16).decode('ascii'),
            'mimeType': 'audio/pcm;rate=16000'}}}))

    async def send_text(self, text):
        """시스템 안내(도착 등)를 사용자 턴으로 주입해 모델이 말하게 한다."""
        if self.closed or not self.ready.is_set():
            return
        await self.ws.write_message(json.dumps({'clientContent': {
            'turns': [{'role': 'user', 'parts': [{'text': text}]}], 'turnComplete': True}}))

    def close(self):
        self.closed = True
        if self.ws:
            self.ws.close()
