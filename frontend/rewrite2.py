import re

def process():
    try:
        with open("Index.html", "r", encoding="utf-8") as f:
            content = f.read()
            
        # 1. Update the overall structure
        # Replace the `body` class and inject the Left Sidebar
        sidebar_html = """<body class="flex flex-row bg-[#0A0A0A] text-white">

<!-- Left Sidebar (New) -->
<aside class="w-[80px] lg:w-[220px] flex flex-col bg-[#111111] border-r border-white/5 shadow-2xl shrink-0 z-20 transition-all">
    <div class="h-[64px] border-b border-white/5 flex items-center justify-center lg:justify-start lg:px-6">
        <div class="font-mono text-xl font-bold tracking-wider text-white">TUNE<span class="text-red-main">_ECU</span></div>
    </div>
    
    <div class="flex-1 py-6 flex flex-col gap-2 px-3 lg:px-4">
        <a href="#" class="flex items-center gap-3 px-3 py-3 rounded-xl bg-red-main/10 text-red-main font-mono text-xs uppercase tracking-wider border border-red-main/20">
            <svg class="w-5 h-5 shrink-0" fill="none" viewBox="0 0 24 24" stroke="currentColor"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M4 6a2 2 0 012-2h2a2 2 0 012 2v2a2 2 0 01-2 2H6a2 2 0 01-2-2V6zM14 6a2 2 0 012-2h2a2 2 0 012 2v2a2 2 0 01-2 2h-2a2 2 0 01-2-2V6zM4 16a2 2 0 012-2h2a2 2 0 012 2v2a2 2 0 01-2 2H6a2 2 0 01-2-2v-2zM14 16a2 2 0 012-2h2a2 2 0 012 2v2a2 2 0 01-2 2h-2a2 2 0 01-2-2v-2z"></path></svg>
            <span class="hidden lg:block">Dashboard</span>
        </a>
        <a href="#" class="flex items-center gap-3 px-3 py-3 rounded-xl text-text-muted hover:text-white hover:bg-white/5 font-mono text-xs uppercase tracking-wider transition-colors">
            <svg class="w-5 h-5 shrink-0" fill="none" viewBox="0 0 24 24" stroke="currentColor"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M12 6V4m0 2a2 2 0 100 4m0-4a2 2 0 110 4m-6 8a2 2 0 100-4m0 4a2 2 0 110-4m0 4v2m0-6V4m6 6v10m6-2a2 2 0 100-4m0 4a2 2 0 110-4m0 4v2m0-6V4"></path></svg>
            <span class="hidden lg:block">Engine</span>
        </a>
        <a href="#" class="flex items-center gap-3 px-3 py-3 rounded-xl text-text-muted hover:text-white hover:bg-white/5 font-mono text-xs uppercase tracking-wider transition-colors">
            <svg class="w-5 h-5 shrink-0" fill="none" viewBox="0 0 24 24" stroke="currentColor"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M8 7h12m0 0l-4-4m4 4l-4 4m0 6H4m0 0l4 4m-4-4l4-4"></path></svg>
            <span class="hidden lg:block">Transmission</span>
        </a>
        <a href="#" class="flex items-center gap-3 px-3 py-3 rounded-xl text-text-muted hover:text-white hover:bg-white/5 font-mono text-xs uppercase tracking-wider transition-colors">
            <svg class="w-5 h-5 shrink-0" fill="none" viewBox="0 0 24 24" stroke="currentColor"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M9 19v-6a2 2 0 00-2-2H5a2 2 0 00-2 2v6a2 2 0 002 2h2a2 2 0 002-2zm0 0V9a2 2 0 012-2h2a2 2 0 012 2v10m-6 0a2 2 0 002 2h2a2 2 0 002-2m0 0V5a2 2 0 012-2h2a2 2 0 012 2v14a2 2 0 01-2 2h-2a2 2 0 01-2-2z"></path></svg>
            <span class="hidden lg:block">Data Logger</span>
        </a>
        <div class="mt-auto">
            <a href="#" class="flex items-center gap-3 px-3 py-3 rounded-xl text-text-muted hover:text-white hover:bg-white/5 font-mono text-xs uppercase tracking-wider transition-colors">
                <svg class="w-5 h-5 shrink-0" fill="none" viewBox="0 0 24 24" stroke="currentColor"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M10.325 4.317c.426-1.756 2.924-1.756 3.35 0a1.724 1.724 0 002.573 1.066c1.543-.94 3.31.826 2.37 2.37a1.724 1.724 0 001.065 2.572c1.756.426 1.756 2.924 0 3.35a1.724 1.724 0 00-1.066 2.573c.94 1.543-.826 3.31-2.37 2.37a1.724 1.724 0 00-2.572 1.065c-.426 1.756-2.924 1.756-3.35 0a1.724 1.724 0 00-2.573-1.066c-1.543.94-3.31-.826-2.37-2.37a1.724 1.724 0 00-1.065-2.572c-1.756-.426-1.756-2.924 0-3.35a1.724 1.724 0 001.066-2.573c-.94-1.543.826-3.31 2.37-2.37.996.608 2.296.07 2.572-1.065z"></path><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M15 12a3 3 0 11-6 0 3 3 0 016 0z"></path></svg>
                <span class="hidden lg:block">Settings</span>
            </a>
        </div>
    </div>
</aside>

<!-- MAIN DASHBOARD WRAPPER -->
<div class="flex-1 flex flex-col min-w-0 bg-[#0A0A0A]">
"""
        
        content = re.sub(r'<body.*?>', sidebar_html, content, count=1)
        content = content.replace('</body>', '\n</div>\n</body>')

        # Replace top Header with Nav
        top_nav_html = """<header class="h-[64px] shrink-0 border-b border-white/5 bg-[#0a0a0a]/80 backdrop-blur-md px-6 flex items-center justify-between z-10 w-full">
    <!-- Top Horizontal Tabs -->
    <div class="flex gap-8 h-full items-end pb-0">
        <div class="pb-[18px] font-mono text-[11px] uppercase tracking-widest text-red-main border-b-2 border-red-main">Telemetry</div>
        <div class="pb-[18px] font-mono text-[11px] uppercase tracking-widest text-text-muted hover:text-white border-b-2 border-transparent cursor-pointer transition-colors">Diagnostics</div>
        <div class="pb-[18px] font-mono text-[11px] uppercase tracking-widest text-text-muted hover:text-white border-b-2 border-transparent cursor-pointer transition-colors">Mapping</div>
        <div class="pb-[18px] font-mono text-[11px] uppercase tracking-widest text-text-muted hover:text-white border-b-2 border-transparent cursor-pointer transition-colors">Library</div>
    </div>
    
    <div class="flex items-center gap-5">
        <div class="text-[11px] font-mono tracking-widest text-text-muted uppercase hidden md:block">Suspension Analysis Engine v2.0</div>
        <div class="h-4 w-px bg-white/10 hidden md:block"></div>
        <div class="flex items-center gap-4 font-mono text-[10px] text-text-muted tracking-widest">
            <div class="flex items-center gap-2">
                <div class="w-1.5 h-1.5 rounded-full bg-red-main shadow-[0_0_8px_rgba(232,64,69,0.6)]" id="statusDot"></div>
                <span id="statusText" class="text-white">READY</span>
            </div>
            <span>|</span>
            <span id="runCount">0 RUNS</span>
            
            <!-- Hidden dropdowns -->
            <select id="designTheme" class="hidden"><option value="aurora"></option></select>
            <select id="layoutMode" class="hidden"><option value="layout-v3"></option></select>
        </div>
    </div>
</header>"""

        # Locate the original header and replace it
        content = re.sub(r'<header.*?</header>', top_nav_html, content, flags=re.DOTALL)

        # Update styling rules
        
        # 4. CARD STYLE REFINEMENT
        # "softer rounded corners (rounded-2xl)", subtle shadow, border white/5, more padding (p-5 or p-6)
        content = content.replace('bg-dark-card border border-dark-border rounded-xl p-4', 'bg-dark-card border border-white/5 rounded-2xl p-6 shadow-lg')
        # Other cards
        content = content.replace('bg-dark-bg border border-dark-border rounded-lg p-3', 'bg-[#111] border border-white/5 rounded-xl p-4 shadow-sm')
        content = content.replace('bg-dark-card border border-dark-border rounded-xl p-2 px-4 shadow-xl', 'bg-[#111] border border-white/5 rounded-2xl p-3 px-5 shadow-lg')
        content = content.replace('bg-dark-card border border-dark-border rounded-xl flex flex-col', 'bg-[#111] border border-white/5 rounded-2xl shadow-lg flex flex-col')
        # Ensure charts keep rounded-2xl
        # History items
        content = content.replace('bg-dark-card border border-dark-border rounded', 'bg-black/40 border border-white/5 rounded-xl hover:bg-[#1f1f1f] transition-colors')
        # History container
        content = content.replace('bg-dark-bg border border-dark-border rounded-lg p-3', 'bg-[#111] border border-white/5 rounded-xl p-4')
        # Profile params
        content = content.replace('bg-dark-bg border border-dark-border rounded-lg p-4 mt-2', 'bg-[#111] border border-white/5 rounded-xl p-5 mt-3 shadow-inner')
        # Overrides inside JS:
        content = content.replace("bg-dark-card border border-dark-border rounded", "bg-black/40 border border-white/5 rounded-xl transition-colors hover:bg-white/5")

        # 6. TYPOGRAPHY IMPROVEMENT
        # Section titles -> uppercase, small tracking
        # Labels -> muted gray (already done, but verify text-sm -> text-xs / text-text-muted)
        content = content.replace('class="text-sm font-medium"', 'class="text-xs uppercase tracking-wider text-text-muted"')
        content = content.replace('class="text-sm"', 'class="text-xs uppercase tracking-wider text-text-muted"')
        content = content.replace('class="font-mono text-xs uppercase tracking-widest text-text-muted mb-2"', 'class="font-mono text-[10px] uppercase tracking-widest text-text-muted mb-3"')
        content = content.replace('text-[10px] text-text-muted uppercase tracking-widest', 'text-[9px] text-text-muted uppercase tracking-widest')
        content = content.replace('font-mono text-2xl font-bold mt-1', 'font-sans font-medium text-3xl mt-1 tracking-tight')

        # 7. BUTTON REFINEMENT
        content = content.replace(
            'bg-red-main hover:bg-red-hover text-white font-mono text-xs px-5 py-2.5 rounded-lg uppercase tracking-wider transition-colors',
            'bg-red-main hover:bg-red-hover text-white font-mono text-xs px-6 py-3 rounded-full uppercase tracking-wider transition-all hover:scale-105 shadow-[0_4px_14px_rgba(232,64,69,0.4)]'
        )
        content = content.replace(
            'bg-white text-dark-bg hover:bg-gray-200 font-mono text-xs px-5 py-2.5 rounded-lg uppercase tracking-wider transition-colors',
            'bg-[#1a1a1a] border border-white/5 text-white hover:bg-[#262626] font-mono text-xs px-6 py-3 rounded-full uppercase tracking-wider transition-all hover:scale-105 shadow-md'
        )

        
        # 8. CHART STYLING
        # Grid -> faint gray. Replace Chart grids in THEME_PALETTES
        content = content.replace(
            "chartGrid: 'rgba(255,255,255,0.05)'",
            "chartGrid: 'rgba(255,255,255,0.03)'"
        )
        # Update point tooltips/hover:
        content = content.replace("tooltipBg: '#171717'", "tooltipBg: '#111111'")

        # Re-write the layout overrides in the setProfile JS to match the softer UI
        content = content.replace(
            "'profile-tab flex-1 py-2 rounded-lg border border-red-main bg-red-main/10 text-white font-mono text-xs tracking-wider transition-colors active'",
            "'profile-tab flex-1 py-2 rounded-xl border border-red-main bg-red-main/10 text-white font-mono text-xs tracking-wider transition-colors active shadow-[0_0_10px_rgba(232,64,69,0.15)]'"
        )
        content = content.replace(
            "'profile-tab flex-1 py-2 rounded-lg border border-dark-border bg-dark-bg text-text-muted hover:text-white font-mono text-xs tracking-wider transition-colors'",
            "'profile-tab flex-1 py-2 rounded-xl border border-white/5 bg-[#111] text-text-muted hover:text-white font-mono text-xs tracking-wider transition-colors'"
        )
        
        # main tabs override
        content = content.replace(
            "'main-tab active px-4 py-2 font-mono text-xs uppercase tracking-widest rounded-lg border border-red-main bg-red-main/10 text-white'",
            "'main-tab active px-5 py-2.5 font-mono text-[10px] uppercase tracking-widest rounded-full border border-red-main bg-red-main/10 text-white shadow-[0_0_10px_rgba(232,64,69,0.1)]'"
        )
        content = content.replace(
            "'main-tab px-4 py-2 font-mono text-xs uppercase tracking-widest rounded-lg border border-transparent text-text-muted hover:text-white'",
            "'main-tab px-5 py-2.5 font-mono text-[10px] uppercase tracking-widest rounded-full border border-transparent text-text-muted hover:text-white hover:bg-white/5 transition-colors'"
        )

        with open("Index.html", "w", encoding="utf-8") as f:
            f.write(content)
            
        print("Success")

    except Exception as e:
        print("Error:", e)

process()
