"""Minimal reader for KiCad s-expression files (pure Python, no KiCad needed).

A node is a Python list: ``[head, arg, arg, ...]``. Bare atoms are ``str``, quoted strings are
``QStr`` (a ``str`` subclass), so ``(locked)`` style flags and ``"locked"`` strings stay apart.
Numbers are left as atoms; convert them where they are used.
"""

from __future__ import annotations

import re
from typing import Iterable, Iterator, List, Optional, Union


class QStr(str):
    """A quoted string from the file (already unescaped)."""

    __slots__ = ()


Node = List[Union[str, "Node"]]

# One token: "(" or ")" or a quoted string (with backslash escapes) or a bare atom.
# A lone '"' (unterminated string) is matched by the last alternative and reported.
_TOKEN = re.compile(r'\(|\)|"(?:[^"\\]|\\.)*"|[^\s()"]+|"', re.S)
_ESCAPE = re.compile(r"\\(.)", re.S)
_ESCAPES = {"n": "\n", "t": "\t", "r": "\r"}


class SexprError(ValueError):
    pass


def _unescape(body: str) -> str:
    if "\\" not in body:
        return body
    return _ESCAPE.sub(lambda m: _ESCAPES.get(m.group(1), m.group(1)), body)


def _atom(tok: str, pos: int) -> str:
    if tok[0] == '"':
        if len(tok) < 2 or tok[-1] != '"':
            raise SexprError(f"unterminated string at offset {pos}")
        return QStr(_unescape(tok[1:-1]))
    return tok


def parse(text: str) -> Node:
    """Parse one complete s-expression (the whole file) and return its root node."""
    stack: List[Node] = []
    root: Optional[Node] = None
    for m in _TOKEN.finditer(text):
        tok = m.group()
        if tok == "(":
            node: Node = []
            if stack:
                stack[-1].append(node)
            elif root is not None:
                raise SexprError(f"second root expression at offset {m.start()}")
            else:
                root = node
            stack.append(node)
        elif tok == ")":
            if not stack:
                raise SexprError(f"unbalanced ')' at offset {m.start()}")
            stack.pop()
        else:
            if not stack:
                raise SexprError(f"atom outside of any list at offset {m.start()}")
            stack[-1].append(_atom(tok, m.start()))
    if stack:
        raise SexprError("unexpected end of input (unclosed '(')")
    if root is None:
        raise SexprError("empty input")
    return root


def iter_top_level(text: str, heads: Iterable[str]) -> Iterator[Node]:
    """Yield the direct children of the root whose head is in ``heads``, in file order.

    Children with other heads (footprints, zones, ...) are skipped without building their
    tree, which keeps large boards fast to scan.
    """
    wanted = frozenset(heads)
    depth = 0
    stack: List[Node] = []  # only non-empty inside a wanted top level child
    expect_head = False  # the next token is the head of a top level child (depth 2)
    for m in _TOKEN.finditer(text):
        tok = m.group()
        if tok == "(":
            depth += 1
            if stack:
                node: Node = []
                stack[-1].append(node)
                stack.append(node)
            else:
                expect_head = depth == 2
        elif tok == ")":
            if depth == 0:
                raise SexprError(f"unbalanced ')' at offset {m.start()}")
            depth -= 1
            expect_head = False
            if stack:
                done = stack.pop()
                if not stack:
                    yield done
        elif depth == 0:
            raise SexprError(f"atom outside of any list at offset {m.start()}")
        elif expect_head:
            expect_head = False
            name = _atom(tok, m.start())
            if name in wanted:
                stack.append([name])
        elif stack:
            stack[-1].append(_atom(tok, m.start()))
        elif tok == '"':
            raise SexprError(f"unterminated string at offset {m.start()}")
        # else: an atom of the root itself or inside a skipped child
    if depth != 0:
        raise SexprError("unexpected end of input (unclosed '(')")


# Accessors ---------------------------------------------------------------------------------


def head(node: object) -> Optional[str]:
    """The head symbol of a list node, or None for atoms and empty lists."""
    if isinstance(node, list) and node and isinstance(node[0], str):
        return node[0]
    return None


def children(node: Node, name: str) -> Iterator[Node]:
    """All direct child lists with the given head."""
    for x in node[1:]:
        if isinstance(x, list) and x and x[0] == name:
            yield x


def child(node: Node, name: str) -> Optional[Node]:
    """The first direct child list with the given head, or None."""
    return next(children(node, name), None)


def atoms(node: Node) -> List[str]:
    """The atom arguments of a node (everything after the head that is not a list)."""
    return [x for x in node[1:] if not isinstance(x, list)]


def has_flag(node: Node, name: str) -> bool:
    """True for a bare flag atom ``(segment locked ...)`` or a child ``(locked yes)``.

    ``(locked no)`` and a quoted ``"locked"`` string do not count.
    """
    for x in node[1:]:
        if isinstance(x, list):
            if x and x[0] == name:
                args = atoms(x)
                return not args or args[0] in ("yes", "true")
        elif x == name and not isinstance(x, QStr):
            return True
    return False


def floats(node: Optional[Node], n: int) -> Optional[tuple]:
    """The first ``n`` atom arguments of ``node`` as floats, or None if missing."""
    if node is None:
        return None
    args = atoms(node)
    if len(args) < n:
        return None
    try:
        return tuple(float(a) for a in args[:n])
    except ValueError:
        return None
