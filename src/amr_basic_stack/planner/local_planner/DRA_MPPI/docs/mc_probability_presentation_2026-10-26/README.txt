발표 자료: MC_probability_comparison_1x_vs_8x.pdf
형식: 16:9, 한국어 8쪽. 한글 글꼴 포함, 텍스트 검색/복사 가능.
구성: 실험 목적 / 실험 방법(환경·MC 재평가) / 결과 및 해석(그림·정확도·시간·한계).

original_1x/probability_comparison.png: 기존 MC 예산(밀도124), 4회 반복으로 재작성.
modified_8x/probability_comparison.png: 8배 MC 예산(밀도992), 동일한 4회 반복.
본문의 전후 수치는 기존 mc_budget_theta_2026-10-23/accuracy.json의 matched 비교다.
최초 1배 원본(8회 반복)과 당시 수정 그림은 historical_png에 별도 보존했다.
slide_01.png ~ slide_08.png: 각 PDF 페이지의 PNG. 슬라이드에 바로 삽입 가능.

대표 궤적의 MC는 후보 CP/비용의 가중평균이 아니다.
현재 split 모드의 1차는 각 guide별 영역과 표본(1024~8192/step)을 사용한다.
각 guide의 가중평균 제어를 제약 적용/rollout한 뒤, 유효 대표 최대5개와 독립 Stop을
하나의 공통 영역에 묶어 별도 final estimator의 새 표본(2048~32768/step)으로 재평가한다.
최종 표본 수는 면적×밀도992를 기준으로 제한하며, 대표 수를 곱하지 않는다.
각 timestep에서 final 표본을 모든 대표가 공유한다. 확률·정적검사·나머지 비용을 새로 계산한다.

측정일 2026-10-23, 자료작성 2026-10-26. 이번 PDF 제작에서 새 실험이나 YAML 변경은 없음.
재현 스크립트: tools/build_mc_probability_pdf.py
출처/소스 hash: manifest.json
글꼴: Google Fonts Nanum Gothic, fonts/OFL.txt 참조.
