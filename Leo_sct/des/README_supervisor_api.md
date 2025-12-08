## GPT-based Supervisor Design Helper

Run `request_supervisor_updates.py` to summarize the current DES data from
`robot_navigation.cpp`, describe the coverage goal, and call the GPT API for a
better supervisor proposal.

```
cd Leo_sct/des
python3 request_supervisor_updates.py --goal "Cover the warehouse aisles"
```

Useful flags:

- `--controllable-events ...`, `--uncontrollable-events ...` – override lists
  from `robot_navigation.cpp` (otherwise parsed automatically).
- `--no-current` – hide the existing transitions, so GPT designs a supervisor
  purely from the provided event list (states still auto-detected from the
  source file).
- `--dry-run` – print the generated prompt without calling the API.
- `--save path.json` – store the returned JSON alongside the console output.
- `--model`, `--temperature` – tweak the OpenAI Chat Completions options.

The script expects an API key via `OPENAI_API_KEY` or `../api_key.txt`. The
response JSON contains `transitions` lines (also echoed verbatim at the end)
that can be pasted back into `robot_navigation.cpp`, plus rationale/coverage
notes for validation.
