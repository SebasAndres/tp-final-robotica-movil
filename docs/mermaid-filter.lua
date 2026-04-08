function CodeBlock(el)
  if el.classes[1] == "mermaid" then
    return pandoc.RawBlock("html", '<div class="mermaid">\n' .. el.text .. '\n</div>')
  end
end
