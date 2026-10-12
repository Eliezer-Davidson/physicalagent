#!/usr/bin/env python3
"""Pre-publication check for this repository.

Run it before pushing anything public:

    python tools/publish_check.py            # from the repo root
    python tools/publish_check.py --verbose  # also list what was scanned

It fails (exit 1) on:
  * secrets and credential files,
  * site network addresses, hardware addresses and SSIDs,
  * developer home directories (any ``/home/<user>`` other than the robots'
    own vendor accounts),
  * identifiers of any second party, person or private workspace, the
    premises phrases that imply one, and the party words that name a second
    party (an organisation, a co-worker, an owner, a brief, an approver),
  * references to other, unpublished repositories, and to any other
    implementation, paper or licence the design could be traced to --
    including any manuscript-server id and any repository URL off the vendor list,
  * ownership and commissioned-work assertions, and a named rights-holder,
  * hardware media files that were removed from the export,
  * agent/session tooling that must never ship,
  * a README image or relative link that does not resolve, and a code-span
    path citation that does not resolve,
  * any file over 10 MB,
  * a missing LICENSE or THIRD_PARTY_NOTICES.md.

DESIGN RULE, learned the hard way: **this file must not spell the words it
forbids.** A checker that lists the private names it is looking for becomes
the leak the moment it ships. So:

  * private identifiers (host names, SSIDs, personal and second-party names,
    site address prefixes) are never present here in any form -- each line's
    tokens are hashed and compared against SHA-256 digests only;
  * the generic patterns (party words, premises phrases, provenance phrases,
    ownership phrases) are assembled from fragments at run time;
  * no needle is ever printed: a hit reports the file, the line number and a
    stable label, never the matched text;
  * the discipline is checked, not trusted: ``check_self`` runs every needle
    and every digest over this file's own text and fails on any hit.

The same generic-rule technique is used by tests/test_paths.py.
"""
from __future__ import annotations

import argparse
import hashlib
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
MAX_FILE_BYTES = 10 * 1024 * 1024

BINARY_SUFFIXES = {".png", ".jpg", ".jpeg", ".gif", ".webp", ".mp4", ".webm",
                   ".pdf", ".so", ".o", ".bin", ".pt", ".pth", ".onnx",
                   ".engine", ".safetensors", ".zip", ".gz", ".tar"}

# Directories that are never scanned for text (they cannot exist in a clean
# export, and the forbidden-path check below fails on them separately).
SKIP_DIRS = {".git", ".venv", "venv", "__pycache__", ".pytest_cache",
             ".mypy_cache", ".ruff_cache", "node_modules"}

# This file quotes its own rules, so it must exempt itself from the text scan.
SELF = pathlib.Path(__file__).resolve()


def _j(*parts: str) -> str:
    """Join fragments. Needles are built here so the literals never appear."""
    return "".join(parts)


def _needles() -> list[tuple[str, re.Pattern[str]]]:
    """(label, compiled pattern). Labels are what gets printed; never the text.

    Everything is matched case-INSENSITIVELY unless the pattern is listed in
    CASE_SENSITIVE below. A previous version compiled these case-sensitively and
    a lowercase spelling of a private repository name walked straight through.
    """
    pats: list[tuple[str, str]] = [
        # --- developer home directories: any /home/<user> that is not one of the
        #     robots' own vendor accounts. Generic on purpose: no machine or user
        #     name has to be spelled here for the rule to work. ------------------
        ("developer home directory",
         _j("/ho", "me/") + r"(?!hiwonder\b|pi\b)[A-Za-z0-9_.-]+"),
        # --- other, unpublished repositories, and this one's private name ----
        ("private repository name", _j("PRI", "SM")),
        ("private repository name", _j("pr", "ism", "_pipe", "line")),
        ("private repository name", _j("Video", "2Real")),
        ("private repository name", _j("MONO", "LITH")),
        ("source repository name", _j("gemini_", "hiwonder_", "bridge")),
        ("private knowledge store", _j("vault", " note")),
        ("umbrella-project framing", r"\b" + _j("pil", "lar") + r"[- ][123]\b"),
        # --- other implementations, papers and licences the design could be
        #     traced to; the work is described in its own terms instead ---------
        # Hyphenated and spaced spellings too: "x-3-y", "x 3y", "x3y".
        ("other-implementation name", r"\b" + _j("re", "st") + r"[- ]?3[- ]?" + _j("d") + r"\b"),
        ("other-implementation paper id", _j("2605", r"\.", "303", "38")),
        ("other-implementation author citation", r"\b" + _j("ma", ",? et", " al") + r"\b"),
        ("manuscript-server reference", r"\b" + _j("ar", "xiv") + r"\b"),
        # Any manuscript-server id of the YYMM.NNNNN form, 2022-2027, not just the one
        # the design could be traced to; and the word that introduces one.
        ("manuscript-server identifier", r"\b" + _j("2[2-7]", r"[01]\d") + r"\.\d{4,5}(?:v\d)?\b"),
        ("manuscript-server word", r"\b" + _j("pre", "print") + r"s?\b"),
        # A repository URL outside the vendor allow-list: vendor repositories are
        # cited on purpose; anyone else's is provenance. The lookbehind spares
        # git-lfs pointer lines ("git-lfs.github.com/spec/v1").
        ("repository URL outside the vendor allow-list",
         r"(?<![\w.-])github\.com/(?!(?:unitreerobotics|NVlabs|NVIDIA|"
         r"eclipse-cyclonedds|Hiwonder)/)[A-Za-z0-9_.-]+/"),
        ("provenance phrase", _j("reference ", "(?:implementation|run|checkout)")),
        ("provenance phrase", _j("external ", "reference")),
        ("provenance phrase", _j("behavio", "u?ral ", "baseline")),
        ("provenance phrase", _j("published ", "(?:method|paper)")),
        ("other-implementation licence", r"\bCC[ -]" + _j("BY", "-NC")),
        ("other-implementation licence", _j("Creative Commons ", "BY", "-NC")),
        # The long-form rewording of the same licence name.
        ("other-implementation licence", _j("non-", "commercial ", "Creative ", "Commons")),
        ("other-implementation licence", _j("Creative ", "Commons") + r"[^\n]{0,40}" + _j("non-", "commercial")),
        # --- party / premises phrases (a vendor name is fine; a place is not) ---
        ("enrolment word", r"\b" + _j("int", "ern", "ship") + r"\b"),
        ("enrolment word", r"\b" + _j("int", "ern") + r"(?:ing|ed|s)?\b"),
        ("enrolment word", r"\b" + _j("prog", "ramme") + r"s?\b"),
        ("party word", r"\b" + _j("emplo", "yer") + r"s?\b"),
        ("party word", r"\b" + _j("men", "tor") + r"s?\b"),
        ("party word", r"\b" + _j("colle", "ague") + r"s?\b"),
        ("party word", r"\b" + _j("instit", "ution") + r"s?\b"),
        ("party word", r"\b" + _j("univers", "it(?:y|ies)") + r"\b"),
        # "Co., Ltd" in a vendor notice is not this word; the word itself is.
        ("party word", r"\b" + _j("comp", "any") + r"\b"),
        # A named second party who owns the robots and gives instructions --
        # also in identifier form (an underscore for the space).
        # "respective owners" and "rights-holder" are untouched by design.
        ("party word", r"(?<![A-Za-z0-9])the[ _]" + _j("own", "er") + r"(?:'s|_s)?(?![A-Za-z0-9])"),
        # A commissioning document, an organisational role, a concealed signatory.
        ("party phrase", r"\bproject " + _j("br", "ief") + r"s?\b"),
        ("party phrase", r"\bproject " + _j("le", "ad") + r"s?\b"),
        ("party phrase", _j("name ", "withheld")),
        ("party phrase", _j("internal ", "deliverable")),
        ("premises phrase", _j("supervised ", "lab")),
        ("premises phrase", r"\b" + _j("lab", "[- ](?:session|machine|workstation|network|hostname)") + r"s?\b"),
        ("premises phrase", r"(?<!isaac )\b" + _j("(?:the|our) ", "lab") + r"(?:'s)?\b"),
        ("premises phrase", r"\b" + _j("the ", "work", "place") + r"\b"),
        # An organisational role over the author. "the client" and "the manager"
        # are left out on purpose: both are ordinary words in a code base (a BLE
        # client, a context manager); the possessive forms are not.
        ("party phrase", r"\b(?:my|his|her) " + _j("(?:super", "visor|man", "ager|bo", "ss|cli", "ent|spon", "sor)") + r"s?\b"),
        ("party phrase", r"\bthe " + _j("(?:bo", "ss|spon", "sor)") + r"\b"),
        # A robot that belongs to someone else: "lent", "borrowed", "on loan"
        # within a short distance of the word robot, in either order.
        ("borrowed-access framing", r"\b" + _j("(?:le", "nt|borr", "owed|on lo", "an)") + r"\b[^\n]{0,30}\brobots?\b"),
        ("borrowed-access framing", r"\brobots?\b[^\n]{0,30}\b" + _j("(?:le", "nt|borr", "owed|on lo", "an)") + r"\b"),
        ("premises phrase", r"\b" + _j("in this ", "lab") + r"\b"),
        ("premises phrase", r"\b(?:enterprise|" + _j("comp", "any") + r") " + _j("wi", "-?fi") + r"\b"),
        ("premises phrase", r"\b" + _j("the ", "office") + r"\b"),
        # Time-boxed access to someone else's machine: the phrase that contrasts
        # the author's home with the robot's site.
        ("borrowed-access framing", r"\b" + _j("at ", "home") + r"\b"),
        # --- ownership assertions: neutral authorship is fine, possession is not --
        ("ownership phrase", r"(?:his|her|their|my|author's) " + _j("own ", "time")),
        ("ownership phrase", r"(?:his|her|their|my|author's) " + _j("own ", "(?:equipment|resources)")),
        ("ownership phrase", _j("work", "[- ]?(?:made[- ])?", "for", "[- ]hire")),
        # A named creator: "created by" followed by a capitalised word (case
        # matters here, so it is listed in CASE_SENSITIVE). A lower-case object
        # ("created by a local install") is not a person.
        ("named creator", r"\b" + _j("creat", "ed by ") + r"[A-Z][a-z]+"),
        ("ownership phrase", r"\bthe " + _j("auth", "or retains") + r"\b"),
        ("ownership phrase", r"(?:his|her|their|my|author's) " + _j("own ", "work")),
        ("ownership phrase", r"\b" + _j("own ", "(?:time|resources|equipment)") + r"\b"),
        ("ownership phrase", r"\b" + _j("hobby ", "project") + r"\b"),
        # A copyright line naming anyone but a vendor on the allow-list.
        ("named rights-holder outside the vendor list",
         r"copyright \(c\) \d{4}(?:\s?[-\u2013,]\s?\d{4})?,? +"
         r"(?!(?:unitree|nvidia|hangzhou|google|meta|hiwonder|eclipse|intel|"
         r"microsoft|the )\b)[A-Za-z]"),
        # --- hardware media removed from the export; must not come back --------
        ("removed hardware media file", _j("g1_air", "step_20aug")),
        ("removed hardware media file", _j("g1_wa", "ve_20aug")),
        ("removed hardware media file", _j("g1-end-of", "-a-hard-day")),
        # --- agent / session tooling that must not be referenced -------------
        ("agent tooling reference", _j("super", "powers")),
        ("agent tooling reference", _j("skill-", "observation")),
        ("agent tooling reference", _j("sub", "agent")),
        ("agent tooling reference", _j("deep-", "research")),
        ("agent tooling reference", _j("Web", "Search") + r"\b"),
        ("agent tooling reference", _j("Web", "Fetch") + r"\b"),
        ("agent tooling reference", _j("memory", " note")),
        # --- internal documents that are deliberately not exported -----------
        ("citation of a document not in this repository",
         r"(?<![A-Za-z0-9-])" + _j("PROM", "PT") + r"\.md\b"),
        ("citation of a document not in this repository", _j("SONIC-G1-", "HANDOFF")),
        ("citation of a document not in this repository",
         r"(?<![A-Za-z0-9])" + _j("e", "du") + r"/"),
        # --- network and hardware addresses ---------------------------------
        # Site address prefixes are matched by digest (see PRIVATE_DIGESTS);
        # only the shape-based rules live here.
        # The canonical placeholder AA:BB:CC:11:22:33 (either separator) is
        # excluded by the leading negative lookahead -- a placeholder is the fix,
        # not a finding.
        ("BLE/MAC address",
         r"\b(?!AA[:_]BB[:_]CC[:_]11[:_]22[:_]33)(?:[0-9A-Fa-f]{2}[:_]){5}"
         r"[0-9A-Fa-f]{2}\b"),
        # --- SSIDs (named ones are matched by digest; this is the shape rule) --
        ("device-serial SSID", r"\bHW-(?!X{8})[0-9A-F]{8}\b"),
        # --- secrets ---------------------------------------------------------
        ("API key literal", r"AIza[0-9A-Za-z_\-]{20,}"),
        ("private key block", r"-----BEGIN [A-Z ]*PRIVATE KEY-----"),
        # The optional closing quote before the separator is what lets this
        # catch a JSON key ("password": "...") as well as an assignment.
        ("credential assignment with a value",
         r"\b(?:api[_-]?key|secret|passwd|password|token)\"?"
         r"\s*[=:]\s*[\"']?[A-Za-z0-9_\-]{8,}[\"']?"),
        # A *default* credential in code is the specific thing this repository
        # got wrong once: a vendor factory pair shipped as a fallback while
        # .env.example said none shipped. An empty default is the fix.
        ("credential default in code",
         r"ssh[_-]?(?:pass|user)\b[^\n]{0,80}getenv\([^)\n]*,\s*[\"'][^\"'\n]+[\"']"),
        ("credential default in code",
         r"SSH_(?:PASS|USER)\s*:[-=]\s*[^}\s]+"),
    ]
    # Patterns whose own case carries meaning: widening them would only add
    # false positives.
    CASE_SENSITIVE = {r"AIza[0-9A-Za-z_\-]{20,}",
                      r"-----BEGIN [A-Z ]*PRIVATE KEY-----",
                      r"\bHW-(?!X{8})[0-9A-F]{8}\b",
                      r"\b" + _j("creat", "ed by ") + r"[A-Z][a-z]+"}
    return [(label, re.compile(p, 0 if p in CASE_SENSITIVE else re.IGNORECASE))
            for label, p in pats]


# SHA-256 digests of private identifiers, lower-cased: site host names, site
# SSIDs, personal and second-party identifiers, and site address prefixes.
# The plaintext is kept nowhere in this repository; the digests were computed
# outside it. A line is tokenised (whole tokens, their alphanumeric pieces, and
# adjacent-piece bigrams; IPv4 addresses contribute their /16, /24 and full
# forms) and every candidate is hashed and looked up here.
PRIVATE_DIGESTS: dict[str, str] = {
    "d2109f6d541f5a91176c3a1f3d90ad5f679e8bda5ceabe79f5764d4e6ca4478b": "site hostname",
    "36dc87df78c853bf91958c2a3d76b1bfb29b448be464b35b81f0125403994c67": "site hostname",
    "beaf4b0df3c39aaca090ef23911ee9190fb0f9985bebcec4fe25988997fe2800": "site hostname",
    "ab877737b6081bd1adaf7cae0f5d18f34c5a8523e399bd672e0f80de2b95a502": "second-party identifier",
    "16522e6439b9fe8c106602f0fdd7caab3de7dda1bea2c05228bb55bffdb8b52a": "second-party identifier",
    "da35859e12fd67fbb9e427c27616b9964969afa386845b87a5056abec6ec5352": "second-party identifier",
    "f338535f78d5b5ce8a582597883138e70c873de55e8d2b3a89e779909a5dca4a": "private workspace identifier",
    "9df31bb1480b22dbb03a108e302ee0d096d59aa132459c262dc4c11e18ec3753": "private account identifier",
    "9bd91b5cce070bd233b55ab04848b1f5fda0e7a0a9da5315c9c828db0df19f7d": "personal identifier",
    "733c0de9bb34380988d22095b3d0faa61075e16c3c4796e24d57fa0fd63e45e7": "personal identifier",
    "1eb4df46c9048ac91598b30804f4de24c458d54d2f0063b1e8bd4b45d234630d": "personal identifier",
    "8810ab3167e88a4881e703f8107d4df58b51825f25c11cae11009d49bdef6b9a": "personal identifier",
    "27037fccea3062ee8ebaea07a9e2bf8dcb6511fd860ae993442aee0c512b8bbf": "personal identifier",
    "4097a5ebb927cde7da6134c472158912f81aae9b94b552462a7a87e175e2beb9": "second-party identifier",
    "7541db43f959bd72027e20f3013f601cf31d5199c85e69262464eb1f96344b54": "second-party identifier",
    "86081f74974697317a51fdd772dc9c20cdfcccfa3d703425f6be65ce8f15ef9c": "second-party identifier",
    "6793252cb65c9dc72913929d5d78359c8d669193e1eb4b3afd8102fd9c347e8c": "second-party identifier",
    "c7673784267fd4ca61e2efa3cfc7529b188d0d6698eb5cb189470ef74984fa31": "second-party identifier",
    "6807b0cde36e926cf922f2cd32705a25617d7a1beb3ddb42c3b28957f83941f4": "second-party identifier",
    "2efb4c15b078751ed182f826d5fc3179d1927eefe27a0bfa46cc1c91e5447898": "site SSID",
    "0ea06f00c7bf0cb98488bf998c21842b16bc3df5d6779fc1aa8b379069b4c252": "personal hotspot SSID",
    "2ee65f45dab89138a681324bfbc2e914dd03ffad6d88509eb3e6d081201565b7": "site LAN address",
    "3d3c3c37e3a64f43fdbad5c2f32504885ceb9cce204cb9778c20e9e28a2d232d": "site LAN address",
    "533786a332f49ec826f680f17cb48e203ee16c9d93f59de24fb4000dc6157ec7": "site LAN address",
    "f77c47905b18c192929fd1c05ccdc28ae68c937b53a7feeaf65d534c6d9b8471": "site LAN subnet",
    "dcc90dea1cbde188aed6cd4570e5f8bf3fc89a2b6235ade16c3ceac4c7f1ce52": "robot DHCP lease",
    "89fcadc408e9a076969de87f5a44ca3de3ef75a3d24aa58ade25b8148cfc1905": "tailscale/CGNAT address",
}

_TOKEN = re.compile(r"[a-z0-9][a-z0-9_.-]*")
_PIECE = re.compile(r"[a-z0-9]+")
_IPV4 = re.compile(r"\b(\d{1,3})\.(\d{1,3})\.(\d{1,3})\.(\d{1,3})\b")


def _candidates(line: str):
    """Every string a private identifier could be hiding in, lower-cased."""
    low = line.lower()
    for m in _TOKEN.findall(low):
        yield m.strip(".-")
    pieces = _PIECE.findall(low)
    for a in pieces:
        yield a
    for a, b in zip(pieces, pieces[1:]):
        yield a + " " + b
    for o in _IPV4.findall(low):
        yield ".".join(o[:2])
        yield ".".join(o[:3])
        yield ".".join(o)


def _private_labels(line: str) -> set[str]:
    hits = set()
    for c in _candidates(line):
        label = PRIVATE_DIGESTS.get(hashlib.sha256(c.encode()).hexdigest())
        if label:
            hits.add(label)
    return hits


# Files that legitimately contain a pattern the scanner would otherwise flag,
# with the reason. Keep this list empty if at all possible.
ALLOW: dict[str, str] = {}

FORBIDDEN_PATHS = [
    "CLAUDE.md", ".env", ".mcp.json", "HANDOFF.md", "NEXT-SESSION.md",
    _j("PROM", "PT") + ".md", "last-review-date.txt",
]
FORBIDDEN_DIRS = [
    ".claude", ".agents", ".cursor", _j("skill-", "observations"), "." + _j("super", "powers"),
    ".venv", "venv", "docs/" + _j("super", "powers"), _j("e", "du"),
]
FORBIDDEN_GLOBS = ["*HANDOFF*.md", "*-PROMPT*.md", "NEXT-SESSION*.md",
                   "MORNING_REPORT*.md", "*.key", "*.pem",
                   # hardware photographs / clips removed from the export
                   _j("g1_*_", "20aug.*"), _j("g1-end-of", "-a-hard-day.*")]

REQUIRED = ["LICENSE", "THIRD_PARTY_NOTICES.md", "README.md"]


def iter_files():
    for p in ROOT.rglob("*"):
        if not p.is_file():
            continue
        rel = p.relative_to(ROOT)
        if any(part in SKIP_DIRS for part in rel.parts):
            continue
        yield p, rel


def check_text(failures, verbose):
    needles = _needles()
    scanned = 0
    for p, rel in iter_files():
        if p.resolve() == SELF:
            continue
        if p.suffix.lower() in BINARY_SUFFIXES:
            continue
        if rel.as_posix() in ALLOW:
            continue
        try:
            text = p.read_text(errors="strict")
        except (UnicodeDecodeError, OSError):
            continue
        scanned += 1
        lines = text.splitlines()
        for label, rx in needles:
            for i, line in enumerate(lines, 1):
                if rx.search(line):
                    failures.append(f"{rel}:{i}: {label}")
        for i, line in enumerate(lines, 1):
            for label in sorted(_private_labels(line)):
                failures.append(f"{rel}:{i}: {label}")
    if verbose:
        print(f"  scanned {scanned} text files")


def check_self(failures):
    """The fragment discipline is enforced, not trusted: every needle and every
    digest is run over this file's own text, with the SELF exemption removed.
    A needle whose fragments were split carelessly, or a comment that spells a
    forbidden phrase, fails the gate here instead of shipping."""
    lines = SELF.read_text(errors="replace").splitlines()
    rel = SELF.relative_to(ROOT)
    for label, rx in _needles():
        for i, line in enumerate(lines, 1):
            if rx.search(line):
                failures.append(f"{rel}:{i}: {label} (the gate spells its own needle)")
    for i, line in enumerate(lines, 1):
        for label in sorted(_private_labels(line)):
            failures.append(f"{rel}:{i}: {label} (the gate spells its own needle)")


def check_paths(failures):
    for name in FORBIDDEN_PATHS:
        if (ROOT / name).exists():
            failures.append(f"{name}: forbidden file is present")
    for name in FORBIDDEN_DIRS:
        if (ROOT / name).exists():
            failures.append(f"{name}/: forbidden directory is present")
    for pattern in FORBIDDEN_GLOBS:
        for p in ROOT.rglob(pattern):
            rel = p.relative_to(ROOT)
            if any(part in SKIP_DIRS for part in rel.parts):
                continue
            failures.append(f"{rel}: matches a forbidden pattern")
    # A virtualenv must never be exported, under any name.
    for p in ROOT.rglob("pyvenv.cfg"):
        failures.append(f"{p.relative_to(ROOT)}: a virtualenv was exported")


def check_required(failures):
    for name in REQUIRED:
        if not (ROOT / name).is_file():
            failures.append(f"{name}: required file is missing")


def check_sizes(failures):
    for p, rel in iter_files():
        n = p.stat().st_size
        if n > MAX_FILE_BYTES:
            failures.append(f"{rel}: {n/1e6:.1f} MB exceeds the 10 MB limit")


_MD_LINK = re.compile(r"!?\[[^\]]*\]\(([^)\s]+)")
_HTML_SRC = re.compile(r'<img[^>]+src="([^"]+)"')


def check_links(failures, verbose):
    checked = 0
    for p, rel in iter_files():
        if p.suffix.lower() != ".md":
            continue
        base = p.parent
        text = p.read_text(errors="replace")
        for rx in (_MD_LINK, _HTML_SRC):
            for target in rx.findall(text):
                if target.startswith(("http://", "https://", "mailto:", "#")):
                    continue
                target = target.split("#")[0]
                if not target:
                    continue
                checked += 1
                if not (base / target).exists():
                    failures.append(f"{rel}: link does not resolve -> {target}")
    if verbose:
        print(f"  checked {checked} relative links")


_CODE_SPAN_PATH = re.compile(
    r"`((?:docs|tests|scripts|tools|physicalagent|simbridge|third_party|batteries|"
    r"robot_nodes)/[A-Za-z0-9_./-]+\.(?:md|py|sh|txt|json|jsonl|png|jpg|patch|yaml|c))`")


def check_code_span_paths(failures):
    """A file path quoted in a code span must exist, relative to the repo root or
    to the citing file. A write-up that names a file this export does not ship
    is the quiet way to lose a reader's trust."""
    for p, rel in iter_files():
        if p.suffix.lower() != ".md":
            continue
        for i, line in enumerate(p.read_text(errors="replace").splitlines(), 1):
            for target in _CODE_SPAN_PATH.findall(line):
                if (ROOT / target).exists() or (p.parent / target).exists():
                    continue
                failures.append(f"{rel}:{i}: cites a path that is not in this repository")


# Capture trees that live offline and are named as such in THIRD_PARTY_NOTICES.md.
OFFLINE_ARTIFACT_DIRS = ("artifacts/hw/", "artifacts/live/", "artifacts/session3/")

_ARTIFACT_CITE = re.compile(r"`(artifacts/[A-Za-z0-9_./<>*{},-]+)`")


def check_artifact_citations(failures):
    """A cited artifact path must exist, be a glob/placeholder, or be declared
    offline. A dangling citation is how a reader loses trust in the rest."""
    for p, rel in iter_files():
        if p.suffix.lower() != ".md":
            continue
        for i, line in enumerate(p.read_text(errors="replace").splitlines(), 1):
            for target in _ARTIFACT_CITE.findall(line):
                if any(c in target for c in "*<>{}"):
                    continue
                if (target.rstrip("/") + "/").startswith(OFFLINE_ARTIFACT_DIRS):
                    continue
                if not (ROOT / target).exists():
                    failures.append(
                        f"{rel}:{i}: cites an artifacts/ path that does not exist "
                        f"and is not declared offline")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    failures: list[str] = []
    print(f"publish_check: {ROOT}")
    check_required(failures)
    check_self(failures)
    check_paths(failures)
    check_sizes(failures)
    check_text(failures, args.verbose)
    check_links(failures, args.verbose)
    check_code_span_paths(failures)
    check_artifact_citations(failures)

    if failures:
        print(f"\nFAIL — {len(failures)} finding(s):")
        for f in failures:
            print(f"  {f}")
        print("\n(The matched text is never printed. Open the file and line "
              "shown, and look for the category named.)")
        return 1
    print("PASS — nothing to fix.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
