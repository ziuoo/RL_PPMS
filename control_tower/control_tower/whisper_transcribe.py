#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Whisper 음성 인식 전용 스크립트
coverage 충돌을 피하기 위해 별도 프로세스에서 실행
"""

import sys
import json

def main():
    if len(sys.argv) != 2:
        print(json.dumps({"error": "Usage: whisper_transcribe.py <audio_file>"}))
        sys.exit(1)
    
    audio_file = sys.argv[1]
    
    try:
        import whisper
        
        model = whisper.load_model("base")
        result = model.transcribe(audio_file, language="ko")
        
        print(json.dumps({"text": result["text"]}))
        sys.exit(0)
        
    except Exception as e:
        print(json.dumps({"error": str(e)}))
        sys.exit(1)

if __name__ == "__main__":
    main()
